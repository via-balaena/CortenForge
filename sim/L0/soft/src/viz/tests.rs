//! Unit tests for the viz primitives (extracted verbatim from the
//! `viz` module — see [`super`] for the primitives under test).

#![allow(
    // Tests use unwrap/expect/panic freely on values whose
    // unwrap-safety is the test's invariant; they're not user code.
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::match_wildcard_for_single_variants,
    // `cast_precision_loss` on len() for averaging in the
    // winding-regression test; len is small (handful of vertices).
    clippy::cast_precision_loss,
    // `float_cmp` on values that have just been `.round()`'d to an
    // integer in the C2 categorical tests; comparing the rounded
    // result against a small integer literal is exact by
    // construction (integer-valued f64 has no fp-noise window).
    clippy::float_cmp
)]

use super::*;
use crate::material::MaterialField;
use crate::mesh::{HandBuiltTetMesh, SingleTetMesh};

/// Single-tet boundary surface: 4 vertices, 4 boundary faces, all
/// per-vertex scalars equal the (single) per-tet scalar.
#[test]
fn boundary_surface_single_tet() {
    let field = MaterialField::skeleton_default();
    let mesh = SingleTetMesh::new(&field);
    let psi = [42.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = boundary_surface(&mesh, &scalars).unwrap();
    assert_eq!(attr.geometry.vertices.len(), 4);
    assert_eq!(attr.geometry.faces.len(), 4);
    let psi_extra = attr.extras.get("psi").expect("psi extra missing");
    assert_eq!(psi_extra.len(), 4);
    for &v in psi_extra {
        assert!((v - 42.0).abs() < 1e-9);
    }
}

/// Two-tet shared-face mesh: 5 vertices, 6 boundary faces (8 total
/// face slots − 2 interior shared faces). Per-vertex psi for the
/// 2 shared-face vertices is the volume-weighted average of both
/// tets' values; for the 3 unique vertices it's the host tet's
/// value.
#[test]
fn boundary_surface_two_tet_shared_face() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let psi: [f64; 2] = [1.0, 3.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = boundary_surface(&mesh, &scalars).unwrap();
    assert_eq!(attr.geometry.vertices.len(), 5);
    assert_eq!(attr.geometry.faces.len(), 6);
    let psi_extra = attr.extras.get("psi").expect("psi extra missing");
    assert_eq!(psi_extra.len(), 5);
}

/// C2 categorical (suffix `_id`): every per-vertex value must be
/// EXACTLY integer. Pre-C2.1 the volume-weighted-per-vertex average
/// at the 3 shared-face vertices of `two_tet_shared_face` would
/// have produced a fractional value mid-way between the two zones;
/// the C2 nearest-tet (k=1, largest-volume incident tet) path
/// emits exactly one of the two input integers at every vertex.
#[test]
fn boundary_surface_categorical_id_stays_integer() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let zone_id: [f64; 2] = [3.0, 7.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("material_zone_id", &zone_id);

    let attr = boundary_surface(&mesh, &scalars).unwrap();
    let zone_extra = attr
        .extras
        .get("material_zone_id")
        .expect("zone extra missing");
    assert_eq!(zone_extra.len(), 5);

    for &v in zone_extra {
        let rounded = v.round();
        assert!(
            (v - rounded).abs() < 1e-9,
            "categorical value {v} is fractional — C2 nearest-tet \
             path should keep every value integer",
        );
        assert!(
            rounded == 3.0 || rounded == 7.0,
            "categorical value {rounded} is not one of the input \
             integers {{3, 7}}",
        );
    }
}

/// C2 mixed scalars: when continuous (`psi`) and categorical
/// (`material_zone_id`) are emitted in the same call, the
/// continuous slot must be BIT-EXACT identical to a continuous-
/// only call (so the C2.1 split doesn't perturb the volume-
/// weighted-per-vertex pipeline), and the categorical slot stays
/// integer.
#[test]
fn boundary_surface_mixed_continuous_and_categorical_preserves_continuous() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let psi: [f64; 2] = [1.0, 3.0];
    let zone_id: [f64; 2] = [3.0, 7.0];

    // Continuous-only baseline.
    let mut cont_only: BTreeMap<&str, &[f64]> = BTreeMap::new();
    cont_only.insert("psi", &psi);
    let baseline = boundary_surface(&mesh, &cont_only).unwrap();

    // Mixed.
    let mut mixed: BTreeMap<&str, &[f64]> = BTreeMap::new();
    mixed.insert("psi", &psi);
    mixed.insert("material_zone_id", &zone_id);
    let attr = boundary_surface(&mesh, &mixed).unwrap();

    // Continuous slot bit-exact preserved.
    let psi_baseline = baseline.extras.get("psi").expect("psi baseline");
    let psi_mixed = attr.extras.get("psi").expect("psi mixed");
    assert_eq!(
        psi_baseline, psi_mixed,
        "continuous `psi` slot must be bit-exact identical when a \
         categorical scalar is added to the call",
    );

    // Categorical slot stays integer.
    let zone = attr
        .extras
        .get("material_zone_id")
        .expect("zone extra missing");
    for &v in zone {
        assert!(
            (v - v.round()).abs() < 1e-9,
            "categorical value {v} fractional",
        );
    }
}

/// Per-tet scalar with wrong length surfaces as the typed error.
#[test]
fn boundary_surface_rejects_wrong_length_scalar() {
    let field = MaterialField::skeleton_default();
    let mesh = SingleTetMesh::new(&field);
    let psi = [1.0_f64, 2.0]; // length 2, mesh has 1 tet
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let err = boundary_surface(&mesh, &scalars).unwrap_err();
    match err {
        VizError::PerTetScalarLengthMismatch {
            name,
            expected,
            actual,
        } => {
            assert_eq!(name, "psi");
            assert_eq!(expected, 1);
            assert_eq!(actual, 2);
        }
        other => panic!("unexpected error: {other:?}"),
    }
}

/// `slab_cut` on `SingleTetMesh` (canonical decimeter tet at the
/// origin with edge 0.1) cut by the x = 0.05 plane. The plane
/// crosses through 1 vertex (`v_1` = (0.1, 0, 0)) on the +x side
/// and 3 vertices on the -x side, producing a single triangle.
#[test]
fn slab_cut_single_tet_one_three_case() {
    let field = MaterialField::skeleton_default();
    let mesh = SingleTetMesh::new(&field);
    let psi = [7.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = slab_cut(
        &mesh,
        Plane {
            axis: 0,
            value: 0.05,
        },
        &scalars,
    )
    .unwrap();
    assert_eq!(attr.geometry.faces.len(), 1);
    assert_eq!(attr.geometry.vertices.len(), 3);

    let psi_extra = attr.extras.get("psi").expect("psi extra missing");
    // All cross-points sit on edges between mesh vertices, all of
    // which carry psi = 7.0 (single-tet → uniform per-vertex).
    // Linear interp between two equal values is the same value.
    for &v in psi_extra {
        assert!((v - 7.0).abs() < 1e-9);
    }
}

/// C2 categorical (suffix `_id`): every cut-vertex value must be
/// EXACTLY integer. Pre-C2.1 the linear interp along each crossed
/// edge between vol-weighted-per-vertex averages would have
/// produced fractional values at any cut vertex on an edge shared
/// between the two zones; the C2 nearest-tet (k=1, largest-volume
/// tet wins on edge dedup) path emits one of the input integers
/// at every cut vertex.
#[test]
fn slab_cut_categorical_id_stays_integer() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let zone_id: [f64; 2] = [3.0, 7.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("material_zone_id", &zone_id);

    // z = 0.05 crosses both tets — tet 0 (corner) in a (1, 3)
    // pattern (v3 above), tet 1 (apex jut) in a (2, 2) pattern
    // (v3 + v4 above). Both contribute cut vertices on the
    // shared edges (v1, v3) and (v2, v3), which dedup to one cut
    // vertex per shared edge whose owner is the larger-volume tet.
    let attr = slab_cut(
        &mesh,
        Plane {
            axis: 2,
            value: 0.05,
        },
        &scalars,
    )
    .unwrap();
    let zone = attr
        .extras
        .get("material_zone_id")
        .expect("zone extra missing");
    assert!(
        !zone.is_empty(),
        "z = 0.05 must produce a non-empty cross-section on \
         two_tet_shared_face",
    );
    for &v in zone {
        let rounded = v.round();
        assert!(
            (v - rounded).abs() < 1e-9,
            "categorical cut-vertex value {v} is fractional — \
             C2 nearest-tet path should keep every value integer",
        );
        assert!(
            rounded == 3.0 || rounded == 7.0,
            "categorical cut-vertex value {rounded} is not one \
             of the input integers {{3, 7}}",
        );
    }
}

/// Winding regression: every cut triangle's normal projects
/// non-negatively onto the cut plane axis.
#[test]
fn slab_cut_winding_aligned_with_axis() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let psi: [f64; 2] = [1.0, 1.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    for axis in 0..3 {
        // Pick a value that's mid-mesh by inspecting positions.
        let value =
            mesh.positions().iter().map(|p| p[axis]).sum::<f64>() / mesh.positions().len() as f64;
        let attr = slab_cut(&mesh, Plane { axis, value }, &scalars).unwrap();
        for face in &attr.geometry.faces {
            let p0 = attr.geometry.vertices[face[0] as usize];
            let p1 = attr.geometry.vertices[face[1] as usize];
            let p2 = attr.geometry.vertices[face[2] as usize];
            let a = p1 - p0;
            let b = p2 - p0;
            let cross_on_axis = match axis {
                0 => a.y.mul_add(b.z, -(a.z * b.y)),
                1 => a.z.mul_add(b.x, -(a.x * b.z)),
                _ => a.x.mul_add(b.y, -(a.y * b.x)),
            };
            assert!(
                cross_on_axis >= 0.0,
                "cut polygon at axis={axis} value={value} has inward-facing normal: \
                 cross_on_axis={cross_on_axis}",
            );
        }
    }
}

/// `slab_cut_deformed` with zero displacement matches `slab_cut`
/// bit-exact (positions identical, scalars identical, face count
/// identical). Mismatched displacement length surfaces as the
/// typed error.
#[test]
fn slab_cut_deformed_zero_displacement_matches_rest() {
    let field = MaterialField::skeleton_default();
    let mesh = SingleTetMesh::new(&field);
    let psi = [7.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let n_v = mesh.n_vertices();
    let zero_disp = vec![Vec3::zeros(); n_v];
    let plane = Plane {
        axis: 0,
        value: 0.05,
    };

    let rest = slab_cut(&mesh, plane, &scalars).unwrap();
    let deformed = slab_cut_deformed(&mesh, plane, &scalars, &zero_disp, 1.0).unwrap();

    assert_eq!(
        rest.geometry.vertices.len(),
        deformed.geometry.vertices.len()
    );
    assert_eq!(rest.geometry.faces.len(), deformed.geometry.faces.len());
    for (a, b) in rest
        .geometry
        .vertices
        .iter()
        .zip(deformed.geometry.vertices.iter())
    {
        assert!((a.x - b.x).abs() < 1e-12);
        assert!((a.y - b.y).abs() < 1e-12);
        assert!((a.z - b.z).abs() < 1e-12);
    }

    // Mismatched displacement length trips the typed error.
    let bad_disp = vec![Vec3::zeros(); 2];
    let err = slab_cut_deformed(&mesh, plane, &scalars, &bad_disp, 1.0).unwrap_err();
    match err {
        VizError::PerVertexLengthMismatch { expected, actual } => {
            assert_eq!(expected, 4);
            assert_eq!(actual, 2);
        }
        other => panic!("unexpected error: {other:?}"),
    }
}

/// Uniform displacement translates the cross-section by
/// `displacement * amplify` along each axis, leaving topology
/// (vertex count, face count) and the in-plane shape intact.
#[test]
fn slab_cut_deformed_uniform_displacement_translates_cross_section() {
    let field = MaterialField::skeleton_default();
    let mesh = SingleTetMesh::new(&field);
    let psi = [7.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let n_v = mesh.n_vertices();
    // Translate in +z so the x = 0.05 cross-section is unchanged
    // in (x, y) but shifted in z by displacement * amplify.
    let uniform = Vec3::new(0.0, 0.0, 0.001);
    let displacement = vec![uniform; n_v];
    let amplify = 5.0_f64;

    let plane = Plane {
        axis: 0,
        value: 0.05,
    };
    let rest = slab_cut(&mesh, plane, &scalars).unwrap();
    let deformed = slab_cut_deformed(&mesh, plane, &scalars, &displacement, amplify).unwrap();

    assert_eq!(rest.geometry.faces.len(), deformed.geometry.faces.len());
    assert_eq!(
        rest.geometry.vertices.len(),
        deformed.geometry.vertices.len()
    );
    let expected_dz = uniform.z * amplify;
    for (a, b) in rest
        .geometry
        .vertices
        .iter()
        .zip(deformed.geometry.vertices.iter())
    {
        assert!((b.x - a.x).abs() < 1e-9);
        assert!((b.y - a.y).abs() < 1e-9);
        assert!(
            (b.z - a.z - expected_dz).abs() < 1e-9,
            "expected dz={expected_dz}, got {}",
            b.z - a.z
        );
    }
}

/// Plane axis out of range surfaces as the typed error.
#[test]
fn slab_cut_rejects_invalid_axis() {
    let field = MaterialField::skeleton_default();
    let mesh = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let err = slab_cut(
        &mesh,
        Plane {
            axis: 3,
            value: 0.0,
        },
        &scalars,
    )
    .unwrap_err();
    match err {
        VizError::InvalidPlaneAxis { axis } => assert_eq!(axis, 3),
        other => panic!("unexpected error: {other:?}"),
    }
}

/// `design_slab_cut` of a sphere at z = 0 produces a triangulated
/// disk whose total area approximates π r² within discretization
/// tolerance. SDF query path validates marching-squares-filled.
#[test]
fn design_slab_cut_disk_area_approximates_pi_r_squared() {
    use crate::SphereSdf;

    let radius = 0.05_f64;
    let sphere = SphereSdf { radius };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    // 80×80 grid over [−0.10, +0.10]² gives ~3% relative area error.
    let resolution = 0.0025_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = design_slab_cut(
        &sphere,
        &analysis,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &bounds,
        resolution,
        &scalars,
    )
    .unwrap();

    let area: f64 = attr
        .geometry
        .faces
        .iter()
        .map(|face| {
            let p0 = attr.geometry.vertices[face[0] as usize];
            let p1 = attr.geometry.vertices[face[1] as usize];
            let p2 = attr.geometry.vertices[face[2] as usize];
            let a = p1 - p0;
            let b = p2 - p0;
            0.5 * a.cross(&b).norm()
        })
        .sum();

    let expected = std::f64::consts::PI * radius * radius;
    let rel_err = (area - expected).abs() / expected;
    assert!(
        rel_err < 0.05,
        "disk area {area} vs expected {expected} (rel_err {rel_err})",
    );
}

/// Plane outside the SDF's interior produces an empty mesh — no
/// triangles, no panics, no errors.
#[test]
fn design_slab_cut_empty_outside_sdf_interior() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    // Plane at z = 0.08 is strictly outside the sphere of radius 0.05.
    let attr = design_slab_cut(
        &sphere,
        &analysis,
        Plane {
            axis: 2,
            value: 0.08,
        },
        &bounds,
        0.005,
        &scalars,
    )
    .unwrap();

    assert_eq!(attr.geometry.faces.len(), 0);
    assert_eq!(attr.geometry.vertices.len(), 0);
    let psi_extra = attr.extras.get("psi").expect("psi extra missing");
    assert_eq!(psi_extra.len(), 0);
}

/// Plane axis out of range surfaces as the typed error.
#[test]
fn design_slab_cut_rejects_invalid_axis() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let err = design_slab_cut(
        &sphere,
        &analysis,
        Plane {
            axis: 7,
            value: 0.0,
        },
        &bounds,
        0.005,
        &scalars,
    )
    .unwrap_err();
    match err {
        VizError::InvalidPlaneAxis { axis } => assert_eq!(axis, 7),
        other => panic!("unexpected error: {other:?}"),
    }
}

/// Resolution must be strictly positive — zero, negative, and `NaN`
/// all surface as the typed error.
#[test]
fn design_slab_cut_rejects_invalid_resolution() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    for bad_res in [0.0_f64, -0.001, f64::NAN] {
        let err = design_slab_cut(
            &sphere,
            &analysis,
            Plane {
                axis: 2,
                value: 0.0,
            },
            &bounds,
            bad_res,
            &scalars,
        )
        .unwrap_err();
        assert!(
            matches!(err, VizError::InvalidResolution { .. }),
            "bad_res = {bad_res}: expected InvalidResolution, got {err:?}",
        );
    }
}

/// Per-tet scalar with wrong length surfaces as the typed error.
#[test]
fn design_slab_cut_rejects_wrong_length_scalar() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    // SingleTetMesh has 1 tet; supply length 2 to trip the gate.
    let psi = [1.0_f64, 2.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let err = design_slab_cut(
        &sphere,
        &analysis,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &bounds,
        0.005,
        &scalars,
    )
    .unwrap_err();
    match err {
        VizError::PerTetScalarLengthMismatch {
            name,
            expected,
            actual,
        } => {
            assert_eq!(name, "psi");
            assert_eq!(expected, 1);
            assert_eq!(actual, 2);
        }
        other => panic!("unexpected error: {other:?}"),
    }
}

/// Saddle-case (2-opposite-corner) resolution: the test uses a
/// pair of disjoint spheres synthesised via `min(sphere_a, sphere_b)`.
/// At a cell where opposite corners are inside (one in each sphere)
/// and adjacent corners are outside, the cell-center SDF is positive
/// (between spheres → outside), so the saddle resolves to TWO
/// disjoint triangles, not a connecting hexagon. Verifies the
/// 0b0101 / 0b1010 dispatch's center-eval path.
#[test]
fn design_slab_cut_saddle_resolves_via_center_eval() {
    // Two spheres centered at (-d, 0, 0) and (+d, 0, 0), both
    // radius r, with d > r so they don't overlap.
    struct TwoSpheres {
        r: f64,
        d: f64,
    }
    impl Sdf for TwoSpheres {
        fn eval(&self, p: NaPoint3<f64>) -> f64 {
            let a = (p - NaPoint3::new(-self.d, 0.0, 0.0)).norm() - self.r;
            let b = (p - NaPoint3::new(self.d, 0.0, 0.0)).norm() - self.r;
            a.min(b)
        }
        fn grad(&self, _p: NaPoint3<f64>) -> Vec3 {
            Vec3::z()
        }
    }
    let two_spheres = TwoSpheres { r: 0.02, d: 0.04 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = design_slab_cut(
        &two_spheres,
        &analysis,
        Plane {
            axis: 2,
            value: 0.0,
        },
        &bounds,
        0.005,
        &scalars,
    )
    .unwrap();

    // Total area should be ~2 × π × r² for two disks.
    let area: f64 = attr
        .geometry
        .faces
        .iter()
        .map(|face| {
            let p0 = attr.geometry.vertices[face[0] as usize];
            let p1 = attr.geometry.vertices[face[1] as usize];
            let p2 = attr.geometry.vertices[face[2] as usize];
            let a = p1 - p0;
            let b = p2 - p0;
            0.5 * a.cross(&b).norm()
        })
        .sum();
    let expected = 2.0 * std::f64::consts::PI * 0.02 * 0.02;
    let rel_err = (area - expected).abs() / expected;
    assert!(
        rel_err < 0.10,
        "two-disk area {area} vs expected {expected} (rel_err {rel_err})",
    );
}

/// `design_surface` of a sphere produces a triangulated boundary
/// whose total area approximates 4πR² within discretization
/// tolerance. Validates marching cubes on the iso-0 surface +
/// scalar transfer integration.
#[test]
fn design_surface_sphere_area_approximates_4_pi_r_squared() {
    use crate::SphereSdf;

    let radius = 0.05_f64;
    let sphere = SphereSdf { radius };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    // 40^3 grid over [-0.10, +0.10]^3 gives ~3% relative area error.
    let resolution = 0.005_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();

    let area: f64 = attr
        .geometry
        .faces
        .iter()
        .map(|face| {
            let p0 = attr.geometry.vertices[face[0] as usize];
            let p1 = attr.geometry.vertices[face[1] as usize];
            let p2 = attr.geometry.vertices[face[2] as usize];
            let a = p1 - p0;
            let b = p2 - p0;
            0.5 * a.cross(&b).norm()
        })
        .sum();

    let expected = 4.0 * std::f64::consts::PI * radius * radius;
    let rel_err = (area - expected).abs() / expected;
    assert!(
        rel_err < 0.05,
        "sphere area {area} vs expected {expected} (rel_err {rel_err})",
    );
}

/// C1: `design_surface` populates per-vertex normals from the SDF
/// gradient. On a sphere the analytical outward normal is the
/// unit position vector `p / |p|`, so each emitted vertex on the
/// iso-0 surface should have a stored normal aligned with its
/// position (within MC discretization tolerance — vertices are
/// linearly interpolated along grid edges, so they sit slightly
/// off the true sphere).
#[test]
fn design_surface_sphere_populates_radial_normals() {
    use crate::SphereSdf;

    let radius = 0.05_f64;
    let sphere = SphereSdf { radius };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let resolution = 0.005_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();

    let normals = attr.normals.as_ref().expect("normals populated by C1");
    assert_eq!(normals.len(), attr.geometry.vertices.len());

    // Every normal should be a unit vector and align with its
    // vertex's outward direction. MC edge-linear-interp puts
    // vertices ~`resolution` off the true sphere, so allow a few
    // degrees of misalignment.
    for (v, n) in attr.geometry.vertices.iter().zip(normals.iter()) {
        assert!(
            (n.norm() - 1.0).abs() < 1e-9,
            "normal not unit-length: |n| = {}",
            n.norm(),
        );
        let r = nalgebra::Vector3::new(v.x, v.y, v.z);
        let r_unit = r / r.norm();
        let dot = n.dot(&r_unit);
        assert!(
            dot > 0.95,
            "normal {n:?} not aligned with radial direction {r_unit:?} (dot = {dot})",
        );
    }
}

/// C2 categorical (suffix `_id`): every MC display-vertex value
/// must be EXACTLY integer. Pre-C2.1 the barycentric interp inside
/// the enclosing analysis tet — whose 4 vertices carry vol-weighted
/// averages that differ at the shared-face vertices — would have
/// produced fractional values at every MC vert; the C2 nearest-
/// tet-centroid (k=1, shared across all categorical scalars per
/// probe) path emits exactly one of the input integers.
///
/// The sphere isosurface at radius 0.015 centered at the tet 1
/// centroid (0.045, 0.045, 0.045) sits fully inside the analysis
/// mesh's combined AABB (0..0.1 each axis), so every MC vertex has
/// candidate analysis tets in its 3×3×3 cell window.
#[test]
fn design_surface_categorical_id_stays_integer() {
    use crate::SphereSdf;

    // Sphere offset from origin so the iso-0 surface lies inside
    // the analysis mesh's interior (origin-anchored sphere would
    // collapse to a single point on the analysis-mesh boundary
    // and produce zero MC verts).
    struct OffsetSphere {
        center: Vec3,
        inner: SphereSdf,
    }
    impl crate::sdf_bridge::Sdf for OffsetSphere {
        fn eval(&self, p: nalgebra::Point3<f64>) -> f64 {
            let shifted = nalgebra::Point3::new(
                p.x - self.center.x,
                p.y - self.center.y,
                p.z - self.center.z,
            );
            self.inner.eval(shifted)
        }
        fn grad(&self, p: nalgebra::Point3<f64>) -> Vec3 {
            let shifted = nalgebra::Point3::new(
                p.x - self.center.x,
                p.y - self.center.y,
                p.z - self.center.z,
            );
            self.inner.grad(shifted)
        }
    }

    let sphere = OffsetSphere {
        center: Vec3::new(0.045, 0.045, 0.045),
        inner: SphereSdf { radius: 0.015 },
    };
    let bounds = Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.1, 0.1, 0.1));
    let resolution = 0.005_f64;
    let field = MaterialField::skeleton_default();
    let analysis = HandBuiltTetMesh::two_tet_shared_face(&field);
    let shell_id: [f64; 2] = [3.0, 7.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("material_shell_id", &shell_id);

    let attr = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
    let shell = attr
        .extras
        .get("material_shell_id")
        .expect("shell extra missing");
    assert!(
        !shell.is_empty(),
        "sphere isosurface should produce a non-empty MC mesh \
         within the analysis-mesh AABB",
    );
    for &v in shell {
        let rounded = v.round();
        assert!(
            (v - rounded).abs() < 1e-9,
            "categorical MC-vertex value {v} is fractional — C2 \
             nearest-tet path should keep every value integer",
        );
        assert!(
            rounded == 3.0 || rounded == 7.0,
            "categorical MC-vertex value {rounded} is not one of \
             the input integers {{3, 7}}",
        );
    }
}

/// Plane outside the SDF's interior produces an empty mesh — no
/// triangles, no panics, no errors.
#[test]
fn design_surface_empty_when_bounds_outside_sdf_interior() {
    use crate::SphereSdf;

    // Sphere at origin radius 0.05; bounds well outside (entirely +x
    // octant far from origin) so the SDF is everywhere positive in
    // the grid → no iso-0 crossings.
    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(0.20, 0.20, 0.20), Vec3::new(0.30, 0.30, 0.30));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let attr = design_surface(&sphere, &analysis, &bounds, 0.005, &scalars).unwrap();

    assert_eq!(attr.geometry.faces.len(), 0);
    let psi_extra = attr.extras.get("psi").expect("psi extra missing");
    assert_eq!(psi_extra.len(), attr.geometry.vertices.len());
}

/// Resolution must be strictly positive — zero, negative, and `NaN`
/// all surface as the typed error.
#[test]
fn design_surface_rejects_invalid_resolution() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    for bad_res in [0.0_f64, -0.001, f64::NAN] {
        let err = design_surface(&sphere, &analysis, &bounds, bad_res, &scalars).unwrap_err();
        assert!(
            matches!(err, VizError::InvalidResolution { .. }),
            "bad_res = {bad_res}: expected InvalidResolution, got {err:?}",
        );
    }
}

/// Per-tet scalar with wrong length surfaces as the typed error.
#[test]
fn design_surface_rejects_wrong_length_scalar() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    // SingleTetMesh has 1 tet; supply length 2 to trip the gate.
    let psi = [1.0_f64, 2.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let err = design_surface(&sphere, &analysis, &bounds, 0.005, &scalars).unwrap_err();
    match err {
        VizError::PerTetScalarLengthMismatch {
            name,
            expected,
            actual,
        } => {
            assert_eq!(name, "psi");
            assert_eq!(expected, 1);
            assert_eq!(actual, 2);
        }
        other => panic!("unexpected error: {other:?}"),
    }
}

/// Zero-displacement case: `design_surface_deformed` with all-zero
/// displacement matches `design_surface` bit-for-bit on positions.
#[test]
fn design_surface_deformed_zero_displacement_matches_rest() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let resolution = 0.005_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let displacement = vec![Vec3::zeros(); analysis.n_vertices()];
    let amplify = 1.0_f64;

    let rest = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
    let deformed = design_surface_deformed(
        &sphere,
        &analysis,
        &bounds,
        resolution,
        &scalars,
        &displacement,
        amplify,
    )
    .unwrap();

    assert_eq!(
        rest.geometry.vertices.len(),
        deformed.geometry.vertices.len()
    );
    for (a, b) in rest
        .geometry
        .vertices
        .iter()
        .zip(deformed.geometry.vertices.iter())
    {
        assert!((a.x - b.x).abs() < 1e-12);
        assert!((a.y - b.y).abs() < 1e-12);
        assert!((a.z - b.z).abs() < 1e-12);
    }
}

/// Uniform displacement translates every surface vertex by exactly
/// `displacement * amplify`. Validates that the barycentric interp
/// (and the clipped+renormalized fallback) preserves uniform fields.
/// Uses an offset sphere fitted inside `SingleTetMesh`'s AABB so
/// every surface vertex falls within the tet-grid spatial index's
/// search window (otherwise fallback returns zeros).
#[test]
fn design_surface_deformed_uniform_displacement_translates_uniformly() {
    struct OffsetSphere {
        center: Vec3,
        radius: f64,
    }
    impl Sdf for OffsetSphere {
        fn eval(&self, p: NaPoint3<f64>) -> f64 {
            (Vec3::new(p.x, p.y, p.z) - self.center).norm() - self.radius
        }
        fn grad(&self, _p: NaPoint3<f64>) -> Vec3 {
            Vec3::z()
        }
    }
    // Sphere centered inside SingleTetMesh's AABB ([0, 0.1]^3),
    // small enough to fit entirely within the tet's neighborhood.
    let sphere = OffsetSphere {
        center: Vec3::new(0.04, 0.04, 0.04),
        radius: 0.015,
    };
    let bounds = Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.1, 0.1, 0.1));
    let resolution = 0.0025_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let uniform = Vec3::new(0.001, 0.002, 0.003);
    let displacement = vec![uniform; analysis.n_vertices()];

    for amplify in [1.0_f64, 2.0, 10.0] {
        let rest = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
        let deformed = design_surface_deformed(
            &sphere,
            &analysis,
            &bounds,
            resolution,
            &scalars,
            &displacement,
            amplify,
        )
        .unwrap();
        assert!(
            !rest.geometry.vertices.is_empty(),
            "rest surface should be non-empty"
        );

        let expected = uniform * amplify;
        for (a, b) in rest
            .geometry
            .vertices
            .iter()
            .zip(deformed.geometry.vertices.iter())
        {
            assert!(
                (b.x - a.x - expected.x).abs() < 1e-9,
                "amplify={amplify}: x diff {} expected {}",
                b.x - a.x,
                expected.x
            );
            assert!((b.y - a.y - expected.y).abs() < 1e-9);
            assert!((b.z - a.z - expected.z).abs() < 1e-9);
        }
    }
}

/// Mismatched displacement length surfaces as the typed error.
#[test]
fn design_surface_deformed_rejects_wrong_displacement_length() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    // SingleTetMesh has 4 vertices; supply length 2 to trip the gate.
    let bad_displacement = vec![Vec3::zeros(); 2];

    let err = design_surface_deformed(
        &sphere,
        &analysis,
        &bounds,
        0.005,
        &scalars,
        &bad_displacement,
        1.0,
    )
    .unwrap_err();
    match err {
        VizError::PerVertexLengthMismatch { expected, actual } => {
            assert_eq!(expected, 4);
            assert_eq!(actual, 2);
        }
        other => panic!("unexpected error: {other:?}"),
    }
}

/// Empty `contact_sdfs` produces output equal to `design_surface`
/// on geometry + scalar field, plus a `primitive_id` scalar that's
/// uniform `0.0` across the body.
#[test]
fn design_scene_empty_contacts_matches_design_surface() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let resolution = 0.005_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let body_only = design_surface(&sphere, &analysis, &bounds, resolution, &scalars).unwrap();
    let scene = design_scene(&sphere, &[], &analysis, &bounds, resolution, &scalars).unwrap();

    assert_eq!(
        body_only.geometry.vertices.len(),
        scene.geometry.vertices.len()
    );
    assert_eq!(body_only.geometry.faces.len(), scene.geometry.faces.len());

    let primitive_id = scene
        .extras
        .get("primitive_id")
        .expect("primitive_id missing");
    assert_eq!(primitive_id.len(), scene.geometry.vertices.len());
    for &id in primitive_id {
        assert!(
            (id - 0.0).abs() < 1e-9,
            "expected body primitive_id 0.0, got {id}"
        );
    }
}

/// Two contact primitives produce `primitive_id` values 0.0 (body),
/// 1.0 (first contact), 2.0 (second contact), each in a contiguous
/// vertex range. Other scalars are padded with 0.0 for contacts.
#[test]
fn design_scene_two_contacts_categorical_primitive_id() {
    use crate::SphereSdf;

    // Body sphere + two non-overlapping contact spheres at offsets.
    struct OffsetSphere {
        cx: f64,
        r: f64,
    }
    impl Sdf for OffsetSphere {
        fn eval(&self, p: NaPoint3<f64>) -> f64 {
            let dx = p.x - self.cx;
            dx.mul_add(dx, p.y.mul_add(p.y, p.z * p.z)).sqrt() - self.r
        }
        fn grad(&self, _p: NaPoint3<f64>) -> Vec3 {
            Vec3::z()
        }
    }

    let body = SphereSdf { radius: 0.05 };
    let contact_a = OffsetSphere { cx: 0.07, r: 0.015 };
    let contact_b = OffsetSphere {
        cx: -0.07,
        r: 0.015,
    };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let scene = design_scene(
        &body,
        &[&contact_a as &dyn Sdf, &contact_b as &dyn Sdf],
        &analysis,
        &bounds,
        0.005,
        &scalars,
    )
    .unwrap();

    let primitive_id = scene
        .extras
        .get("primitive_id")
        .expect("primitive_id missing");

    // All three categorical values present.
    let mut counts = [0_usize; 3];
    for &id in primitive_id {
        let bucket = if (id - 0.0).abs() < 0.5 {
            0
        } else if (id - 1.0).abs() < 0.5 {
            1
        } else if (id - 2.0).abs() < 0.5 {
            2
        } else {
            panic!("primitive_id {id} out of expected range 0..=2");
        };
        counts[bucket] += 1;
    }
    assert!(counts[0] > 0, "expected body vertices (id=0)");
    assert!(counts[1] > 0, "expected contact_a vertices (id=1)");
    assert!(counts[2] > 0, "expected contact_b vertices (id=2)");

    // psi extra is body-only-meaningful — padded 0.0 for contacts.
    // Total length matches vertex count.
    let psi_extra = scene.extras.get("psi").expect("psi missing");
    assert_eq!(psi_extra.len(), scene.geometry.vertices.len());
}

/// `design_scene` populates normals over the entire vertex range
/// (body + contacts). The PLY writer enforces this invariant; the
/// pre-helper-refactor code only stamped body normals.
#[test]
fn design_scene_normals_cover_body_and_contacts() {
    use crate::SphereSdf;

    struct OffsetSphere {
        cx: f64,
        r: f64,
    }
    impl Sdf for OffsetSphere {
        fn eval(&self, p: NaPoint3<f64>) -> f64 {
            let dx = p.x - self.cx;
            dx.mul_add(dx, p.y.mul_add(p.y, p.z * p.z)).sqrt() - self.r
        }
        fn grad(&self, p: NaPoint3<f64>) -> Vec3 {
            let dx = p.x - self.cx;
            let d = Vec3::new(dx, p.y, p.z);
            let n = d.norm();
            if n > 1e-12 { d / n } else { Vec3::z() }
        }
    }

    let body = SphereSdf { radius: 0.05 };
    let contact = OffsetSphere { cx: 0.07, r: 0.015 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let scene = design_scene(
        &body,
        &[&contact as &dyn Sdf],
        &analysis,
        &bounds,
        0.005,
        &scalars,
    )
    .unwrap();

    let normals = scene.normals.as_ref().expect("normals slot missing");
    assert_eq!(
        normals.len(),
        scene.geometry.vertices.len(),
        "normals must cover every vertex (body + contacts)"
    );
    for n in normals {
        let nsq = n.x.mul_add(n.x, n.y.mul_add(n.y, n.z * n.z));
        assert!((nsq - 1.0).abs() < 1e-6, "non-unit normal: {nsq}");
    }
}

/// `design_scene_deformed` with empty `contact_sdfs` produces output
/// equal to `design_surface_deformed` on geometry + scalars, plus a
/// `primitive_id` extra that's uniform `0.0` across the body.
#[test]
fn design_scene_deformed_empty_contacts_matches_deformed_surface() {
    use crate::SphereSdf;

    let sphere = SphereSdf { radius: 0.05 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let resolution = 0.005_f64;
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let n_v = analysis.n_vertices();
    let uniform = Vec3::new(0.001, 0.0, 0.0);
    let displacement = vec![uniform; n_v];
    let amplify = 5.0_f64;

    let body_only = design_surface_deformed(
        &sphere,
        &analysis,
        &bounds,
        resolution,
        &scalars,
        &displacement,
        amplify,
    )
    .unwrap();
    let scene = design_scene_deformed(
        &sphere,
        &[],
        &analysis,
        &bounds,
        resolution,
        &scalars,
        &displacement,
        amplify,
    )
    .unwrap();

    assert_eq!(
        body_only.geometry.vertices.len(),
        scene.geometry.vertices.len()
    );
    assert_eq!(body_only.geometry.faces.len(), scene.geometry.faces.len());

    let primitive_id = scene
        .extras
        .get("primitive_id")
        .expect("primitive_id missing");
    assert_eq!(primitive_id.len(), scene.geometry.vertices.len());
    for &id in primitive_id {
        assert!(id.abs() < 1e-9, "expected body primitive_id 0.0, got {id}");
    }
}

/// `design_scene_deformed` with a contact primitive: body verts get
/// `primitive_id = 0.0` and are displaced by `displacement * amplify`;
/// contact verts get `primitive_id = 1.0` and are NOT displaced
/// (contacts are rigid; their geometry comes from the contact SDF's
/// rest pose, set per-call by the caller). Normals slot covers the
/// full merged vertex range.
#[test]
fn design_scene_deformed_one_contact_separates_body_from_contact() {
    use crate::SphereSdf;

    struct OffsetSphere {
        cx: f64,
        r: f64,
    }
    impl Sdf for OffsetSphere {
        fn eval(&self, p: NaPoint3<f64>) -> f64 {
            let dx = p.x - self.cx;
            dx.mul_add(dx, p.y.mul_add(p.y, p.z * p.z)).sqrt() - self.r
        }
        fn grad(&self, p: NaPoint3<f64>) -> Vec3 {
            let dx = p.x - self.cx;
            let d = Vec3::new(dx, p.y, p.z);
            let n = d.norm();
            if n > 1e-12 { d / n } else { Vec3::z() }
        }
    }

    let body = SphereSdf { radius: 0.05 };
    let contact = OffsetSphere { cx: 0.07, r: 0.015 };
    let bounds = Aabb3::new(Vec3::new(-0.10, -0.10, -0.10), Vec3::new(0.10, 0.10, 0.10));
    let field = MaterialField::skeleton_default();
    let analysis = SingleTetMesh::new(&field);
    let psi = [1.0_f64];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("psi", &psi);

    let n_v = analysis.n_vertices();
    let displacement = vec![Vec3::zeros(); n_v];

    let scene = design_scene_deformed(
        &body,
        &[&contact as &dyn Sdf],
        &analysis,
        &bounds,
        0.005,
        &scalars,
        &displacement,
        1.0,
    )
    .unwrap();

    let primitive_id = scene
        .extras
        .get("primitive_id")
        .expect("primitive_id missing");
    let mut n_body = 0_usize;
    let mut n_contact = 0_usize;
    for &id in primitive_id {
        if id.abs() < 0.5 {
            n_body += 1;
        } else if (id - 1.0).abs() < 0.5 {
            n_contact += 1;
        } else {
            panic!("unexpected primitive_id {id}");
        }
    }
    assert!(n_body > 0, "body vertices missing");
    assert!(n_contact > 0, "contact vertices missing");

    let psi_extra = scene.extras.get("psi").expect("psi missing");
    assert_eq!(psi_extra.len(), scene.geometry.vertices.len());

    let normals = scene.normals.as_ref().expect("normals slot missing");
    assert_eq!(
        normals.len(),
        scene.geometry.vertices.len(),
        "normals must cover body + contact"
    );
}

// ---- C2.2 idw_k_nearest_tet_centroids unit tests ----

/// Constant per-tet field → output equals that constant at any
/// probe regardless of distance distribution. Trivially follows
/// from `Σ w_i * c / Σ w_i = c`; pinned as a baseline so future
/// kernel changes don't accidentally bias constant fields.
#[test]
fn idw_k_nearest_constant_field_returns_constant() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let positions = mesh.positions();
    let per_tet: [f64; 2] = [42.5, 42.5];
    let candidates: Vec<u32> = vec![0, 1];
    for probe in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(1.0, -2.0, 3.0),
    ] {
        let out = idw_k_nearest_tet_centroids(
            probe,
            &mesh,
            positions,
            &candidates,
            &[&per_tet],
            8,
            1e-12,
        );
        assert_eq!(out.len(), 1);
        assert!(
            (out[0] - 42.5).abs() < 1e-12,
            "constant field at probe {probe:?}: got {} expected 42.5",
            out[0],
        );
    }
}

/// Single candidate tet → output equals that tet's per-tet value
/// regardless of probe-to-centroid distance. The IDW reduces to
/// `(w * v) / w = v` when only one tet contributes.
#[test]
fn idw_k_nearest_single_candidate_returns_that_tet_value() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let positions = mesh.positions();
    let per_tet: [f64; 2] = [42.0, 99.0];
    // Probe arbitrary, well away from either centroid.
    let probe = Vec3::new(10.0, 10.0, 10.0);
    // Only tet 1 in the candidate set.
    let candidates: Vec<u32> = vec![1];
    let out =
        idw_k_nearest_tet_centroids(probe, &mesh, positions, &candidates, &[&per_tet], 8, 1e-12);
    assert!(
        (out[0] - 99.0).abs() < 1e-12,
        "single-tet IDW must return that tet's value, got {}",
        out[0],
    );
}

/// Probe at the midpoint between two tet centroids with scalars
/// `0.0` and `1.0` → IDW returns `0.5` (equal weights cancel).
/// Pins the equal-distance branch of the Shepard kernel.
#[test]
fn idw_k_nearest_symmetric_two_candidates_returns_average() {
    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let positions = mesh.positions();
    // Tet 0 centroid = (0.025, 0.025, 0.025); tet 1 centroid =
    // mean of v1=(0.1,0,0), v2=(0,0.1,0), v3=(0,0,0.1),
    // v4=(0.08,0.08,0.08) = (0.045, 0.045, 0.045). Midpoint:
    // (0.035, 0.035, 0.035) is equidistant from both centroids.
    let probe = Vec3::new(0.035, 0.035, 0.035);
    let per_tet: [f64; 2] = [0.0, 1.0];
    let candidates: Vec<u32> = vec![0, 1];
    let out =
        idw_k_nearest_tet_centroids(probe, &mesh, positions, &candidates, &[&per_tet], 8, 1e-12);
    assert!(
        (out[0] - 0.5).abs() < 1e-12,
        "equidistant probe IDW must average to 0.5, got {}",
        out[0],
    );
}

/// `design_slab_cut` C2.1 gap-fix: a categorical `_id` scalar on
/// the design-SDF cross-section must produce integer-only output,
/// matching the rest of the C2.1 design-X family. Pre-fix the
/// vol-weighted-per-vertex + barycentric path would produce
/// fractional values at any cross-vertex inside an analysis tet
/// whose 4 vertices carry different integer-derived averages.
#[test]
fn design_slab_cut_categorical_id_stays_integer() {
    // Use the analysis-mesh boundary directly as the design SDF —
    // the cross-section at z=0.05 then samples shell_id from the
    // two analysis tets via nearest_tet_centroid_idx.
    struct MeshBoxSdf;
    impl crate::sdf_bridge::Sdf for MeshBoxSdf {
        fn eval(&self, p: nalgebra::Point3<f64>) -> f64 {
            let lo = nalgebra::Vector3::new(0.0, 0.0, 0.0);
            let hi = nalgebra::Vector3::new(0.1, 0.1, 0.1);
            let q = nalgebra::Vector3::new(
                (lo.x - p.x).max(p.x - hi.x),
                (lo.y - p.y).max(p.y - hi.y),
                (lo.z - p.z).max(p.z - hi.z),
            );
            q.x.max(q.y).max(q.z)
        }
        fn grad(&self, _p: nalgebra::Point3<f64>) -> Vec3 {
            Vec3::z()
        }
    }
    let sdf = MeshBoxSdf;

    let field = MaterialField::skeleton_default();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&field);
    let shell_id: [f64; 2] = [3.0, 7.0];
    let mut scalars: BTreeMap<&str, &[f64]> = BTreeMap::new();
    scalars.insert("material_shell_id", &shell_id);
    let bounds = Aabb3::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.1, 0.1, 0.1));

    let attr = design_slab_cut(
        &sdf,
        &mesh,
        Plane {
            axis: 2,
            value: 0.05,
        },
        &bounds,
        0.01,
        &scalars,
    )
    .unwrap();
    let shell = attr
        .extras
        .get("material_shell_id")
        .expect("shell extra missing");
    assert!(
        !shell.is_empty(),
        "z=0.05 design slab cut must produce a non-empty cross-section",
    );
    for &v in shell {
        let rounded = v.round();
        assert!(
            (v - rounded).abs() < 1e-9,
            "categorical design-slab-cut value {v} is fractional — \
             C2.1 gap-fix should keep every value integer",
        );
        assert!(
            rounded == 3.0 || rounded == 7.0,
            "categorical design-slab-cut value {rounded} is not one \
             of the input integers {{3, 7}}",
        );
    }
}
