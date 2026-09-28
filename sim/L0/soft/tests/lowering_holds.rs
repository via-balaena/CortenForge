//! The lowering's model and holds (`sim_soft::lowering`, plan §16w), on a cup meshed with its cut points located: a
//! spherical shell cut open by a plane, pressed from inside by a sphere through its mouth.
//!
//! The skin is exactly the boundary vertices on the outer sphere; a mount and a rigid shell hold their vertices
//! still through a run while the rest of the wall moves; the lowering names each element's vertices and each node's
//! material; and it refuses what it cannot lower.

// A meshing or a run that fails is the test's panic; held displacements are compared exactly, since a held node's
// displacement is never written.
#![allow(clippy::expect_used, clippy::float_cmp, clippy::cast_precision_loss)]

use nalgebra::{Point3, Vector3};
use sim_soft::lowering::{Lowered, Lowering, LoweringError, Plane, Skin, lower};
use sim_soft::{
    Aabb3, ConstantField, CutPoints, MaterialField, Mesh, MeshingHints, Sdf, SdfMeshedTetMesh,
    SphereSdf, TetId, Vec3, VertexId, Yeoh,
};
use sim_soft_explicit::ModelError;
use sim_soft_explicit::cpu::f64::CpuExecutor;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::f64::{Pose, SdfGridLayout};
use sim_soft_explicit::stepping::{Stepper, StepperConfig};

/// The cup's outer and inner radii, the height of its mouth plane, and its lattice spacing.
const OUTER: f64 = 0.020;
const INNER: f64 = 0.012;
const MOUTH: f64 = 0.004;
const CELL: f64 = 0.004;

/// The pressing sphere's radius; it passes the mouth (the canal's radius there is 11.3 mm) and meets the canal's
/// floor.
const PRESS: f64 = 0.008;

/// The silicone: μ, Yeoh's C₂ (every catalog silicone has one), and a density.
const MU: f64 = 3.0e4;
const C2: f64 = 0.1 * MU;
const DENSITY: f64 = 1_070.0;

/// A spherical shell between [`INNER`] and [`OUTER`], below the plane `z = MOUTH`.
struct Cup;

impl Sdf for Cup {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let r = p.coords.norm();
        (r - OUTER).max(INNER - r).max(p.z - MOUTH)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let r = p.coords.norm();
        let radial = p.coords / r;
        let pieces = [
            (r - OUTER, radial),
            (INNER - r, -radial),
            (p.z - MOUTH, Vector3::z()),
        ];
        pieces
            .into_iter()
            .max_by(|a, b| a.0.total_cmp(&b.0))
            .map_or_else(Vector3::z, |piece| piece.1)
    }
}

/// The cup, meshed with its cut points located, of one Yeoh material with λ = 4μ (the model sets its own λ).
fn cup() -> SdfMeshedTetMesh<Yeoh> {
    let constant = |value: f64| Box::new(ConstantField::new(value));
    let field = MaterialField::from_yeoh_fields(constant(MU), constant(C2), constant(4.0 * MU));
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(-0.023, -0.0217, -0.0229),
            Vec3::new(0.023, 0.023, 0.009),
        ),
        cell_size: CELL,
        material_field: Some(field),
    };
    SdfMeshedTetMesh::from_sdf_yeoh_with(&Cup, &hints, CutPoints::Root).expect("the cup must mesh")
}

/// The skin of [`cup`] against the outer sphere.
fn skin(mesh: &SdfMeshedTetMesh<Yeoh>) -> Skin {
    Skin::of(mesh, &SphereSdf { radius: OUTER }, CELL)
}

const ELASTIC: Lowering = Lowering {
    poisson: 0.45,
    viscous_time: 0.0,
};

/// The pressing sphere, falling from above the mouth to 2 mm into the canal's floor over `loading` seconds.
fn press(loading: f64) -> Obstacle {
    let (cell, half) = (0.001, 0.012);
    let side = 25;
    let mut values = Vec::with_capacity(side * side * side);
    for k in 0..side {
        for j in 0..side {
            for i in 0..side {
                let at = |n: usize| -half + n as f64 * cell;
                values.push(Vector3::new(at(i), at(j), at(k)).norm() - PRESS);
            }
        }
    }
    let height = |z: f64| Pose {
        qw: 1.0,
        qx: 0.0,
        qy: 0.0,
        qz: 0.0,
        tx: 0.0,
        ty: 0.0,
        tz: z,
    };
    let count = 100;
    Obstacle {
        grid: SdfGridLayout {
            origin_x: -half,
            origin_y: -half,
            origin_z: -half,
            cell_size: cell,
            size_x: 25,
            size_y: 25,
            size_z: 25,
        },
        values,
        fine: None,
        start: 0.0,
        interval: loading / f64::from(count),
        poses: (0..=count)
            .map(|k| height(0.030 - 0.036 * f64::from(k) / f64::from(count)))
            .collect(),
        friction: 0.0,
    }
}

/// `lowered` run under [`press`] for its loading and as long again held there: each node's displacement at the end.
fn pressed(lowered: &Lowered) -> Vec<[f64; 3]> {
    let loading = 0.03;
    let obstacle = press(loading);
    let executor = CpuExecutor::new(&lowered.model, &obstacle).expect("a valid obstacle");
    let mut stepper = Stepper::new(executor, StepperConfig::new(0.0), 0.0);
    stepper
        .run_until(2.0 * loading)
        .expect("the run stays finite");
    stepper.executor_mut().snapshot().displacements
}

/// The model's nodes whose source vertex is in `vertices`.
fn nodes_of(lowered: &Lowered, vertices: &[VertexId]) -> Vec<usize> {
    (0..lowered.source.len())
        .filter(|&n| vertices.binary_search(&lowered.source[n]).is_ok())
        .collect()
}

#[test]
fn the_skin_is_the_boundary_on_the_outer_surface_and_nothing_inside_the_wall() {
    let mesh = cup();
    let skin = skin(&mesh);
    let positions = mesh.positions();
    let radius = |v: VertexId| Vec3::norm(&positions[v as usize]);
    let mut boundary: Vec<VertexId> = mesh.boundary_faces().iter().flatten().copied().collect();
    boundary.sort_unstable();
    boundary.dedup();
    let on_the_sphere: Vec<VertexId> = boundary
        .iter()
        .copied()
        .filter(|&v| (radius(v) - OUTER).abs() <= 1e-9 * OUTER)
        .collect();
    assert!(on_the_sphere.len() > 100, "{}", on_the_sphere.len());
    assert!(
        skin.vertices()
            .iter()
            .all(|v| boundary.binary_search(v).is_ok()),
        "a skin vertex inside the wall"
    );
    // Every boundary vertex on the outer sphere, and besides them only vertices in the mouth plane near the rim.
    assert!(
        on_the_sphere
            .iter()
            .all(|v| skin.vertices().binary_search(v).is_ok())
    );
    let rim_radius = OUTER.mul_add(OUTER, -MOUTH * MOUTH).sqrt();
    let bevel: Vec<VertexId> = skin
        .vertices()
        .iter()
        .copied()
        .filter(|v| on_the_sphere.binary_search(v).is_err())
        .collect();
    for &v in &bevel {
        let p = positions[v as usize];
        assert_eq!(
            p.z, MOUTH,
            "vertex {v} is neither on the sphere nor in the mouth"
        );
        assert!(
            rim_radius - p.x.hypot(p.y) < CELL,
            "vertex {v} is more than a cell from the rim"
        );
    }
    assert!(!bevel.is_empty(), "the fixture must show the bevel");
    // The old pin's rule, every vertex within half a cell of the outer surface, takes vertices inside the wall here.
    let mut referenced: Vec<VertexId> = (0..mesh.n_tets())
        .flat_map(|t| mesh.tet_vertices(TetId::try_from(t).expect("a small mesh")))
        .collect();
    referenced.sort_unstable();
    referenced.dedup();
    let inside_the_wall = referenced
        .iter()
        .filter(|v| boundary.binary_search(v).is_err())
        .filter(|&&v| (radius(v) - OUTER).abs() <= 0.5 * CELL)
        .count();
    assert!(
        inside_the_wall > 0,
        "the fixture must show the old rule's defect"
    );
    // The mouth's own faces are not skin: its vertices more than a cell from the rim are not taken.
    let mouth_inside: Vec<VertexId> = boundary
        .iter()
        .copied()
        .filter(|&v| {
            let p = positions[v as usize];
            p.z == MOUTH && rim_radius - p.x.hypot(p.y) > CELL
        })
        .collect();
    assert!(!mouth_inside.is_empty());
    assert!(
        mouth_inside
            .iter()
            .all(|v| skin.vertices().binary_search(v).is_err())
    );
}

#[test]
fn lowering_names_every_elements_vertices_and_the_materials() {
    let mesh = cup();
    let densities = vec![DENSITY; mesh.n_tets()];
    let lowering = Lowering {
        poisson: 0.49,
        viscous_time: 0.25,
    };
    let lowered = lower(&mesh, &densities, lowering, &[]).expect("the cup must lower");
    let model = &lowered.model;
    let mut referenced: Vec<VertexId> = (0..mesh.n_tets())
        .flat_map(|t| mesh.tet_vertices(TetId::try_from(t).expect("a small mesh")))
        .collect();
    referenced.sort_unstable();
    referenced.dedup();
    assert!(
        referenced.len() < mesh.positions().len(),
        "the fixture must leave unreferenced vertices, or dropping them is not exercised"
    );
    assert_eq!(model.node_count(), referenced.len());
    assert_eq!(model.element_count(), mesh.n_tets());
    assert!(model.held().iter().all(|&h| !h));
    for (element, corners) in model.elements().iter().enumerate() {
        let tet = TetId::try_from(element).expect("a small mesh");
        let names: Vec<VertexId> = corners.map(|n| lowered.source[n as usize]).to_vec();
        assert_eq!(names, mesh.tet_vertices(tet).to_vec(), "element {element}");
        let material = model.materials()[element];
        assert_eq!(
            [material.mu, material.c2, material.density],
            [MU, C2, DENSITY]
        );
        assert!((material.lambda / MU - 49.0).abs() < 1e-12, "ν 0.49");
        assert!((material.viscosity / (0.25 * MU) - 1.0).abs() < 1e-15);
    }
    for (node, &vertex) in lowered.source.iter().enumerate() {
        let p = mesh.positions()[vertex as usize];
        assert_eq!(model.rest_positions()[node], [p.x, p.y, p.z]);
    }
}

#[test]
fn each_hold_holds_its_vertices_through_a_press() {
    let mesh = cup();
    let densities = vec![DENSITY; mesh.n_tets()];
    let skin = skin(&mesh);
    let base = Plane::new(Point3::new(0.0, 0.0, -0.014), -Vector3::z()).expect("a plane");
    let mount = skin.beyond(&mesh, &base);
    assert!(!mount.is_empty() && mount.len() < skin.vertices().len());
    for &v in skin.vertices() {
        let below = mesh.positions()[v as usize].z < -0.014;
        assert_eq!(mount.binary_search(&v).is_ok(), below, "vertex {v}");
    }
    let mean_z = |displacements: &[[f64; 3]]| {
        displacements.iter().map(|u| u[2]).sum::<f64>() / displacements.len() as f64
    };

    // Nothing held: the press carries the whole cup down.
    let free = lower(&mesh, &densities, ELASTIC, &[]).expect("the cup must lower");
    let carried = mean_z(&pressed(&free));
    assert!(carried < -1e-4, "the free cup moved {carried} m");

    for (label, held) in [
        ("mounted", mount.as_slice()),
        ("in a shell", skin.vertices()),
    ] {
        let lowered = lower(&mesh, &densities, ELASTIC, held).expect("the cup must lower");
        let nodes = nodes_of(&lowered, held);
        assert_eq!(nodes.len(), held.len(), "{label}");
        for (node, &is_held) in lowered.model.held().iter().enumerate() {
            assert_eq!(
                is_held,
                nodes.binary_search(&node).is_ok(),
                "{label}: node {node}"
            );
        }
        let displacements = pressed(&lowered);
        assert!(
            nodes.iter().all(|&n| displacements[n] == [0.0; 3]),
            "{label}: a held node moved"
        );
        let moved = displacements
            .iter()
            .map(|u| Vector3::from(*u).norm())
            .fold(0.0, f64::max);
        assert!(moved > 1e-4, "{label}: the press moved nothing ({moved} m)");
        assert!(
            mean_z(&displacements) > 0.2 * carried,
            "{label}: the cup was carried off"
        );
    }
}

#[test]
fn what_cannot_be_lowered_is_refused() {
    let mesh = cup();
    let densities = vec![DENSITY; mesh.n_tets()];
    for poisson in [0.5, -0.1, f64::NAN] {
        let lowering = Lowering {
            poisson,
            viscous_time: 0.0,
        };
        assert_eq!(
            lower(&mesh, &densities, lowering, &[]).err(),
            Some(LoweringError::Parameters)
        );
    }
    assert_eq!(
        lower(&mesh, &densities[1..], ELASTIC, &[]).err(),
        Some(LoweringError::Densities {
            found: densities.len() - 1,
            expected: densities.len()
        })
    );
    let mut named: Vec<VertexId> = (0..mesh.n_tets())
        .flat_map(|t| mesh.tet_vertices(TetId::try_from(t).expect("a small mesh")))
        .collect();
    named.sort_unstable();
    let orphan = (0..VertexId::try_from(mesh.positions().len()).expect("a small mesh"))
        .find(|v| named.binary_search(v).is_err())
        .expect("the lattice leaves vertices no element names");
    assert_eq!(
        lower(&mesh, &densities, ELASTIC, &[orphan]).err(),
        Some(LoweringError::Held { vertex: orphan })
    );
    // A density that is not finite is refused as a material, not as two materials meeting.
    let mut broken = densities.clone();
    broken[0] = f64::NAN;
    assert!(matches!(
        lower(&mesh, &broken, ELASTIC, &[]).err(),
        Some(LoweringError::Model(ModelError::InvalidMaterial {
            element: 0,
            ..
        }))
    ));
    // Two densities: two materials meet, and their rule is not chosen.
    let mut two = densities;
    two[mesh.n_tets() / 2] = 2.0 * DENSITY;
    assert_eq!(
        lower(&mesh, &two, ELASTIC, &[]).err(),
        Some(LoweringError::MaterialsMeet {
            element: mesh.n_tets() / 2
        })
    );
}

/// The cup's outer shell with its floor: the sphere, closed by the mouth's plane.
struct ShellWithAFloor;

impl Sdf for ShellWithAFloor {
    fn eval(&self, p: Point3<f64>) -> f64 {
        (p.coords.norm() - OUTER).max(p.z - MOUTH)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        if p.coords.norm() - OUTER > p.z - MOUTH {
            p.coords.normalize()
        } else {
            Vector3::z()
        }
    }
}

#[test]
fn the_skin_needs_the_outer_surfaces_own_field() {
    // Read against the outer shell's field, whose floor closes the mouth, the faces beside the mouth read near zero,
    // the canal's among them: the skin would take vertices on the canal.
    let mesh = cup();
    let radius = |v: VertexId| Vec3::norm(&mesh.positions()[v as usize]);
    let floored = Skin::of(&mesh, &ShellWithAFloor, CELL);
    let on_the_canal = floored
        .vertices()
        .iter()
        .filter(|&&v| (radius(v) - INNER).abs() <= 1e-9 * INNER)
        .count();
    assert!(
        on_the_canal > 0,
        "the floor's field must take the canal's rim"
    );
    assert!(
        skin(&mesh)
            .vertices()
            .iter()
            .all(|&v| (radius(v) - INNER).abs() > 1e-9 * INNER)
    );
}
