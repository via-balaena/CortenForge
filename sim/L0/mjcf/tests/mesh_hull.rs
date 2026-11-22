//! A mesh's convex hull as MuJoCo 3.5.0 makes it in a model, against
//! `tests/assets/mesh_hull/mesh_hull.txt` (written by
//! `sim/L0/tests/scripts/gen_mesh_hull_reference.py` from MuJoCo's own models):
//! made of the vertices as written, before `refpos`, `refquat` and `scale`
//! (`mjCMesh::Process`, `user_mesh.cc:1567-1603`), under `maxhullvert` of
//! qhull's choice of vertices, given as faces to a mesh written without
//! them (`CopyGraph`), and made only for the meshes MuJoCo makes it for: a
//! mesh or SDF geom's mesh where the geom collides or a `<pair>` names it,
//! and a `convex` mesh. Each graph is compared int for int in MuJoCo's
//! `mesh_graph` layout, each mesh's faces id for id, its polygons corner for
//! corner from the same corner, and the body's mass; a document MuJoCo
//! refuses, by MuJoCo's words.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_core::mesh::{MeshHull, MeshPolygons};
use sim_mjcf::load_model;

/// MuJoCo's values were printed on macOS arm64, where a polygon's normal
/// and the mass are compared to the bit; elsewhere within 1e-12 of the
/// value's size (or of 1).
const BITWISE: bool = cfg!(all(target_os = "macos", target_arch = "aarch64"));

/// One document of the golden and what MuJoCo made of it.
struct Case {
    name: String,
    xml: String,
    outcome: Outcome,
}

/// What MuJoCo made of one mesh.
#[derive(Default)]
struct Mesh {
    vertices: usize,
    /// Its `mesh_graph` block; `None` without one.
    graph: Option<Vec<i64>>,
    faces: Vec<u32>,
    /// Each polygon's normal and corners.
    polygons: Vec<([f64; 3], Vec<usize>)>,
}

enum Outcome {
    /// MuJoCo's refusal, its words before the mesh's name.
    Refused(String),
    /// Each mesh and body 1's mass.
    Loaded { meshes: Vec<Mesh>, mass: f64 },
}

fn ints<T: std::str::FromStr>(text: &str) -> Vec<T>
where
    T::Err: std::fmt::Debug,
{
    text.split_whitespace()
        .map(|x| x.parse().unwrap())
        .collect()
}

/// A double from the hex of its bits.
fn double(hex: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(hex, 16).unwrap())
}

fn golden() -> Vec<Case> {
    let text = include_str!("assets/mesh_hull/mesh_hull.txt");
    let mut cases: Vec<Case> = Vec::new();
    let mut meshes: Vec<Mesh> = Vec::new();
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let (key, rest) = line.split_once(' ').unwrap_or((line, ""));
        match key {
            "case" => cases.push(Case {
                name: rest.to_owned(),
                xml: String::new(),
                outcome: Outcome::Refused(String::new()),
            }),
            "xml" => rest.clone_into(&mut cases.last_mut().unwrap().xml),
            "err" => {
                // "mesh volume is negative (misoriented triangles): m": its
                // words before the name.
                let words = rest.rsplit_once(": ").map_or(rest, |(w, _)| w);
                cases.last_mut().unwrap().outcome = Outcome::Refused(words.to_owned());
            }
            "mesh" => {
                let n: Vec<usize> = ints(rest);
                meshes.push(Mesh {
                    vertices: n[1],
                    ..Mesh::default()
                });
            }
            "graph" => meshes.last_mut().unwrap().graph = (rest != "none").then(|| ints(rest)),
            "face" => meshes.last_mut().unwrap().faces = ints(rest),
            "poly" => {
                let words: Vec<&str> = rest.split_whitespace().collect();
                let normal = [double(words[0]), double(words[1]), double(words[2])];
                let corners = words[3..].iter().map(|w| w.parse().unwrap()).collect();
                meshes.last_mut().unwrap().polygons.push((normal, corners));
            }
            "mass" => {
                cases.last_mut().unwrap().outcome = Outcome::Loaded {
                    meshes: std::mem::take(&mut meshes),
                    mass: double(rest),
                };
            }
            other => panic!("golden key {other}"),
        }
    }
    cases
}

/// The hull in MuJoCo's `mesh_graph` layout: the vertex and face counts,
/// `vert_edgeadr`, `vert_globalid`, `edge_localid` (each list ended by -1)
/// and `face_globalid`.
fn to_mujoco(hull: &MeshHull) -> Vec<i64> {
    let count = |n: usize| i64::try_from(n).unwrap();
    let mut adr = Vec::new();
    let mut edges = Vec::new();
    for n in &hull.neighbours {
        adr.push(count(edges.len()));
        edges.extend(n.iter().map(|&e| i64::from(e)));
        edges.push(-1);
    }
    let mut out = vec![count(hull.vertex.len()), count(hull.faces.len())];
    out.extend(adr);
    out.extend(hull.vertex.iter().map(|&v| count(v)));
    out.extend(edges);
    out.extend(hull.faces.iter().flatten().map(|&v| i64::from(v)));
    out
}

/// Whether `ours` is `theirs` as [`BITWISE`] compares; never for a `NaN`.
fn near(ours: f64, theirs: f64) -> bool {
    if BITWISE {
        ours.to_bits() == theirs.to_bits()
    } else {
        (ours - theirs).abs() <= 1e-12 * theirs.abs().max(1.0)
    }
}

/// Where our polygons part from MuJoCo's, or `None`.
fn polygons_part(ours: &MeshPolygons, theirs: &[([f64; 3], Vec<usize>)]) -> Option<String> {
    if ours.paths.len() != theirs.len() {
        return Some(format!("{} polygons / {}", ours.paths.len(), theirs.len()));
    }
    let parts: Vec<String> = ours
        .paths
        .iter()
        .zip(&ours.normals)
        .zip(theirs)
        .enumerate()
        .filter_map(|(k, ((path, normal), (their_normal, corners)))| {
            if path != corners {
                Some(format!("polygon {k}: corners {path:?} / {corners:?}"))
            } else if !(0..3).all(|i| near(normal[i], their_normal[i])) {
                Some(format!("polygon {k}: normal {normal:?} / {their_normal:?}"))
            } else {
                None
            }
        })
        .collect();
    (!parts.is_empty()).then(|| parts.join("; "))
}

#[test]
fn each_meshs_hull_is_mujocos() {
    let cases = golden();
    assert_eq!(cases.len(), 43, "golden cases");
    let mut failures = Vec::new();
    for case in &cases {
        let loaded = load_model(&case.xml);
        match (&case.outcome, loaded) {
            (Outcome::Refused(words), Err(err)) => {
                if !err.to_string().contains(words.as_str()) {
                    failures.push(format!("{}: refused '{err}', MuJoCo '{words}'", case.name));
                }
            }
            (Outcome::Refused(words), Ok(_)) => {
                failures.push(format!("{}: loads, MuJoCo refuses '{words}'", case.name));
            }
            (Outcome::Loaded { .. }, Err(err)) => {
                failures.push(format!("{}: refused '{err}', MuJoCo loads it", case.name));
            }
            (Outcome::Loaded { meshes, mass }, Ok(model)) => {
                assert_eq!(model.mesh_data.len(), meshes.len(), "{}: meshes", case.name);
                for (i, (want, data)) in meshes.iter().zip(&model.mesh_data).enumerate() {
                    let graph = data.hull().map(to_mujoco);
                    let faces: Vec<u32> = data.triangles().iter().flatten().copied().collect();
                    if data.vertex_count() != want.vertices
                        || graph != want.graph
                        || faces != want.faces
                    {
                        failures.push(format!(
                            "{} mesh {i}: vertices {} / {}, graph {} / {}, faces {}",
                            case.name,
                            data.vertex_count(),
                            want.vertices,
                            if graph.is_some() { "made" } else { "none" },
                            if want.graph.is_some() { "made" } else { "none" },
                            if faces == want.faces {
                                "equal"
                            } else {
                                "differ"
                            },
                        ));
                    }
                    if let Some(part) = polygons_part(data.polygons(), &want.polygons) {
                        failures.push(format!("{} mesh {i}: {part}", case.name));
                    }
                }
                if !near(model.body_mass[1], *mass) {
                    failures.push(format!(
                        "{}: mass {} / {mass}",
                        case.name, model.body_mass[1]
                    ));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {}:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
}
