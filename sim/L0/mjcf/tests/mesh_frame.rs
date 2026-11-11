//! A mesh in the frame MuJoCo 3.5.0 compiles it to (book Rigid-loading L35,
//! Q63): its vertices moved to its centre of mass and turned to its
//! principal axes (`mjCMesh::Process`, `user_mesh.cc:1676-1686`), and its
//! geom's frame taking that move in, its size, box and radius from the
//! mesh's box (`mjCGeom::Compile`, `user_objects.cc:3748-3769`).
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-11). Each geom is turned by a written `quat`, made unit by
//! a square root alone, so its bits are the same on every platform.

// The values are MuJoCo's, printed to the last digit, and compared to it.
#![allow(clippy::unwrap_used, clippy::unreadable_literal, clippy::float_cmp)]

use sim_mjcf::load_model;

/// A 2 × 1 × 0.5 box turned 30° about z in its vertex data, centred at
/// (0.3, −0.2, 0.1), its vertices rounded to six places.
const TURNED_BOX: &str = r#"<mesh name="m" vertex="-0.316025 -1.133013 -0.15  1.416025 -0.133013 -0.15  0.916025 0.733013 -0.15  -0.816025 -0.266987 -0.15  -0.316025 -1.133013 0.35  1.416025 -0.133013 0.35  0.916025 0.733013 0.35  -0.816025 -0.266987 0.35" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/>"#;

/// A geom's frame, size, box and radius as MuJoCo stores them: `geom_pos`,
/// `geom_quat` (w, x, y, z), `geom_size`, `geom_aabb` and `geom_rbound`.
#[derive(Debug, PartialEq)]
struct Geom {
    pos: [f64; 3],
    quat: [f64; 4],
    size: [f64; 3],
    aabb: [f64; 6],
    rbound: f64,
}

/// Geom `g` of `xml`'s model.
fn geom(xml: &str, g: usize) -> Geom {
    let model = load_model(xml).unwrap();
    let q = model.geom_quat[g];
    Geom {
        pos: model.geom_pos[g].into(),
        quat: [q.w, q.i, q.j, q.k],
        size: model.geom_size[g].into(),
        aabb: model.geom_aabb[g],
        rbound: model.geom_rbound[g],
    }
}

/// `ours` against MuJoCo's, bit for bit.
fn assert_geom(ours: &Geom, mujoco: &Geom) {
    let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    assert!(
        bits(&ours.pos) == bits(&mujoco.pos)
            && bits(&ours.quat) == bits(&mujoco.quat)
            && bits(&ours.size) == bits(&mujoco.size)
            && bits(&ours.aabb) == bits(&mujoco.aabb)
            && ours.rbound.to_bits() == mujoco.rbound.to_bits(),
        "{ours:?} against MuJoCo's {mujoco:?}"
    );
}

/// The turned box's geom on a free body, posed by `pos` and `quat`.
#[test]
fn mesh_geom_frame_as_mujoco() {
    let xml = format!(
        r#"<mujoco><asset>{TURNED_BOX}</asset><worldbody><body pos="0 0 1"><freejoint/><geom type="mesh" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></worldbody></mujoco>"#
    );
    assert_geom(
        &geom(&xml, 0),
        &Geom {
            pos: [0.4084210623594906, 0.16842104735321844, 0.5094736791192604],
            quat: [
                -0.4117395192948302,
                0.4154224734035923,
                0.3466951528915816,
                0.7332783972023116,
            ],
            size: [0.2500000000000011, 0.5000004236599732, 0.9999999635411352],
            aabb: [
                0.0,
                -7.960015757646488e-09,
                -8.071468160508743e-09,
                0.2500000000000011,
                0.5000004156999575,
                0.999999955469667,
            ],
            rbound: 1.1456440768154936,
        },
    );
}

/// An off-centre tetrahedron's geom in the world body, moved by `pos`.
#[test]
fn a_static_meshs_geom_frame_as_mujoco() {
    let xml = r#"<mujoco><asset><mesh name="m" vertex="0.3 0.4 0.5  1.3 0.4 0.5  0.3 2.4 0.5  0.3 0.4 3.5" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><geom type="mesh" mesh="m" pos="1 0 0"/></worldbody></mujoco>"#;
    assert_geom(
        &geom(xml, 0),
        &Geom {
            pos: [1.5499999970197678, 0.9000000283122062, 1.25],
            quat: [
                0.9729070394130872,
                0.1522921364049869,
                -0.06801202622878003,
                0.16010422273607272,
            ],
            size: [0.5161073119561442, 1.2306677773283157, 2.3047651051426765],
            aabb: [
                -0.046366337804556124,
                0.16219603867415056,
                0.5701104741861923,
                0.4697409741515881,
                1.0684717386541651,
                1.7346546309564843,
            ],
            rbound: 2.66324090639434,
        },
    );
}

/// The turned box's geom in a body `fusestatic` fuses into its parent:
/// MuJoCo fuses after compiling the geom (`user_model.cc:4933-4938`), so the
/// body's frame is accumulated into the geom's with the mesh's in it, where
/// sim-mjcf fuses first and with nalgebra's products (ledger L373, L255).
/// Its position, size, box and radius are MuJoCo's bit for bit; its
/// orientation is within 1e-15 (measured: one unit in the last place in three
/// components; which of the two rows parts it has not been isolated).
#[test]
fn a_fused_meshs_geom_frame_as_mujoco() {
    let xml = format!(
        r#"<mujoco><compiler fusestatic="true"/><asset>{TURNED_BOX}</asset><worldbody><body pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><body pos="0.2 -0.1 0.4" quat="0.8 0.3 0.1 -0.2"><geom type="mesh" mesh="m" pos="0.1 0.2 0.3" quat="0.9 0.1 -0.2 0.3"/></body></body></worldbody></mujoco>"#
    );
    let ours = geom(&xml, 1);
    let mujoco = [
        -0.3872751974180785,
        0.3979761211300948,
        -0.07573330445646463,
        0.8281892266089562,
    ];
    assert!(
        ours.quat
            .iter()
            .zip(mujoco)
            .all(|(a, b)| (a - b).abs() <= 1e-15),
        "{:?} against MuJoCo's {mujoco:?}",
        ours.quat
    );
    assert_geom(
        &Geom {
            quat: mujoco,
            ..ours
        },
        &Geom {
            pos: [0.6642375225172397, -0.4635087752971915, 0.7272334580306792],
            quat: mujoco,
            size: [0.2500000000000011, 0.5000004236599732, 0.9999999635411352],
            aabb: [
                0.0,
                -7.960015757646488e-09,
                -8.071468160508743e-09,
                0.2500000000000011,
                0.5000004156999575,
                0.999999955469667,
            ],
            rbound: 1.1456440768154936,
        },
    );
}

/// `mesh`'s vertices as `xml`'s model holds them against MuJoCo's
/// `mesh_vert`, bit for bit.
fn assert_vertices(xml: &str, mesh: usize, mujoco: &[f64]) {
    let model = load_model(xml).unwrap();
    let ours: Vec<u64> = model.mesh_data[mesh]
        .vertices()
        .iter()
        .flat_map(|v| [v.x, v.y, v.z])
        .map(f64::to_bits)
        .collect();
    let mujoco: Vec<u64> = mujoco.iter().map(|x| x.to_bits()).collect();
    assert_eq!(ours, mujoco);
}

/// A mesh's vertices are moved in double precision and kept in single, as
/// MuJoCo keeps them (`mesh_vert`, `CopyVert`, `user_mesh.cc:892-896`):
/// the turned box's, and the tetrahedron of the census's `9b0854df`.
#[test]
fn embedded_mesh_vertices_are_f32() {
    let xml = format!(
        r#"<mujoco><asset>{TURNED_BOX}</asset><worldbody><body><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#
    );
    assert_vertices(
        &xml,
        0,
        &[
            -0.25,
            0.5000001192092896,
            -0.9999999403953552,
            -0.25,
            0.5000004172325134,
            0.9999993443489075,
            -0.25,
            -0.5000001192092896,
            0.9999999403953552,
            -0.25,
            -0.5000004172325134,
            -0.9999993443489075,
            0.25,
            0.5000001192092896,
            -0.9999999403953552,
            0.25,
            0.5000004172325134,
            0.9999993443489075,
            0.25,
            -0.5000001192092896,
            0.9999999403953552,
            0.25,
            -0.5000004172325134,
            -0.9999993443489075,
        ],
    );
    let tetra = r#"<mujoco><asset><mesh name="tetra" vertex="-0.5 -0.28868 -0.20412  0.5 -0.28868 -0.20412  0.0 0.57735 -0.20412  0.0 0.0 0.61237" face="0 2 1  0 1 3  1 2 3  0 3 2"/></asset><worldbody><body pos="0 0 0.5"><freejoint/><geom type="mesh" mesh="tetra" density="1000"/></body></worldbody></mujoco>"#;
    assert_vertices(
        tetra,
        0,
        &[
            -0.15945158898830414,
            0.5,
            0.31555646657943726,
            -0.15945158898830414,
            -0.5,
            0.31555646657943726,
            -0.2868064045906067,
            0.0,
            -0.5410582423210144,
            0.6057096123695374,
            0.0,
            -0.0900547206401825,
        ],
    );
}
