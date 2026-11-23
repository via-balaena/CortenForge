//! A mesh's octree as MuJoCo 3.5.0 makes it for a signed distance field
//! geom, against `tests/assets/mesh_octree/mesh_octree.txt` (written by
//! `sim/L0/tests/scripts/gen_mesh_octree_reference.py` from MuJoCo's own
//! models).
//!
//! The octree is made over a mesh an SDF geom names, colliding or not, its
//! type written or a class's, on a body or the world, and kept where
//! `discardvisual` drops that geom but another keeps the mesh, as MuJoCo
//! marks the mesh before it discards (`mjCModel::IndexAssets`); none is made
//! for a mesh only mesh geoms or fitted primitives name. Each is made in the
//! frame MuJoCo keeps the mesh in, over meshes moved, turned, scaled,
//! mirrored, tiny and huge, under each inertia mode, written without faces,
//! open, flat, built in, and read from an STL, an OBJ and an MSH file; and
//! over meshes where a tie decides a corner's sign: a flat mesh (a corner in
//! its plane is on neither side of it), two parallel plates (a corner midway
//! is as far from each) and a box wound at random. Each octree is compared by
//! its node count and a hash of each of its four arrays; a
//! file mesh's convex hull graph int for int (ledger L439: no other
//! committed test makes the hull of a mesh read from a file); a document
//! MuJoCo refuses, by MuJoCo's words.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_core::mesh::{MeshHull, MeshOctree};
use sim_mjcf::{load_model, load_model_from_file};

/// MuJoCo's octrees were hashed on macOS arm64. A coefficient's magnitude is
/// a distance made of sums, products and square roots, and its sign the side
/// of a pseudonormal whose vertex weights are angles from the platform's
/// `acos`: there the signed coefficients' hash is compared, elsewhere the
/// magnitudes' hash and how many are negative.
const BITWISE: bool = cfg!(all(target_os = "macos", target_arch = "aarch64"));

/// One document of the golden and what MuJoCo made of it.
struct Case {
    name: String,
    xml: String,
    /// Each mesh file the document names, with its bytes.
    files: Vec<(String, Vec<u8>)>,
    outcome: Outcome,
}

/// What MuJoCo made of one mesh.
struct Mesh {
    vertices: usize,
    faces: usize,
    /// Its `mesh_graph` block, for a file case; `None` without one.
    graph: Option<Vec<i64>>,
    octree: Option<Hashes>,
}

/// An octree's node count, the hashes of its arrays (the golden's header
/// says how they are made) and how many coefficients are negative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Hashes {
    nodes: usize,
    aabb: u64,
    child: u64,
    depth: u64,
    coeff: u64,
    abs_coeff: u64,
    negative: usize,
}

enum Outcome {
    /// MuJoCo's refusal, its words before the mesh's name.
    Refused(String),
    Loaded(Vec<Mesh>),
}

fn ints<T: std::str::FromStr>(text: &str) -> Vec<T>
where
    T::Err: std::fmt::Debug,
{
    text.split_whitespace()
        .map(|x| x.parse().unwrap())
        .collect()
}

fn hex(text: &str) -> u64 {
    u64::from_str_radix(text, 16).unwrap()
}

fn bytes(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).unwrap())
        .collect()
}

fn golden() -> Vec<Case> {
    let text = include_str!("assets/mesh_octree/mesh_octree.txt");
    let mut cases: Vec<Case> = Vec::new();
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let (key, rest) = line.split_once(' ').unwrap_or((line, ""));
        let case = cases.last_mut();
        match key {
            "case" => cases.push(Case {
                name: rest.to_owned(),
                xml: String::new(),
                files: Vec::new(),
                outcome: Outcome::Refused(String::new()),
            }),
            "xml" => rest.clone_into(&mut case.unwrap().xml),
            "file" => {
                let (name, data) = rest.split_once(' ').unwrap();
                case.unwrap().files.push((name.to_owned(), bytes(data)));
            }
            "err" => {
                let words = rest.rsplit_once(": ").map_or(rest, |(w, _)| w);
                case.unwrap().outcome = Outcome::Refused(words.to_owned());
            }
            "meshes" => case.unwrap().outcome = Outcome::Loaded(Vec::new()),
            "mesh" => {
                let n: Vec<usize> = ints(rest);
                let Outcome::Loaded(meshes) = &mut case.unwrap().outcome else {
                    panic!("a mesh outside a loaded case");
                };
                meshes.push(Mesh {
                    vertices: n[1],
                    faces: n[2],
                    graph: None,
                    octree: None,
                });
            }
            "graph" | "octree" => {
                let Outcome::Loaded(meshes) = &mut case.unwrap().outcome else {
                    panic!("a {key} outside a loaded case");
                };
                let mesh = meshes.last_mut().unwrap();
                if key == "graph" {
                    mesh.graph = (rest != "none").then(|| ints(rest));
                } else if rest != "none" {
                    let w: Vec<&str> = rest.split_whitespace().collect();
                    mesh.octree = Some(Hashes {
                        nodes: w[0].parse().unwrap(),
                        aabb: hex(w[1]),
                        child: hex(w[2]),
                        depth: hex(w[3]),
                        coeff: hex(w[4]),
                        abs_coeff: hex(w[5]),
                        negative: w[6].parse().unwrap(),
                    });
                }
            }
            other => panic!("golden key {other}"),
        }
    }
    cases
}

/// The golden's hash of `words`: from 0, each word xored in, then
/// splitmix64's finalizer.
fn mix_hash(words: impl Iterator<Item = u64>) -> u64 {
    words.fold(0, |h, w| {
        let mut z = h ^ w;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    })
}

/// An int's word: the int sign-extended.
#[allow(clippy::cast_sign_loss)] // the golden's word is the int's bits
fn int_word(x: i32) -> u64 {
    i64::from(x) as u64
}

fn hashes(octree: &MeshOctree) -> Hashes {
    let coeff = || octree.coeff.iter().flatten();
    Hashes {
        nodes: octree.len(),
        aabb: mix_hash(octree.aabb.iter().flatten().map(|x| x.to_bits())),
        child: mix_hash(octree.child.iter().flatten().map(|&x| int_word(x))),
        depth: mix_hash(octree.depth.iter().map(|&x| int_word(x))),
        coeff: mix_hash(coeff().map(|x| x.to_bits())),
        abs_coeff: mix_hash(coeff().map(|x| x.abs().to_bits())),
        negative: coeff().filter(|&&x| x < 0.0).count(),
    }
}

/// Whether `ours` is MuJoCo's as [`BITWISE`] compares.
fn same_octree(ours: &Hashes, theirs: &Hashes) -> bool {
    if BITWISE {
        ours == theirs
    } else {
        Hashes {
            coeff: theirs.coeff,
            ..*ours
        } == *theirs
    }
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

/// The case loaded as MuJoCo loaded it: from its text, or, with mesh
/// files, from a directory holding the document and its files.
fn load(case: &Case) -> sim_mjcf::Result<sim_core::Model> {
    if case.files.is_empty() {
        return load_model(&case.xml);
    }
    let dir = tempfile::tempdir().unwrap();
    for (name, data) in &case.files {
        std::fs::write(dir.path().join(name), data).unwrap();
    }
    let path = dir.path().join("doc.xml");
    std::fs::write(&path, &case.xml).unwrap();
    load_model_from_file(&path)
}

#[test]
fn each_meshs_octree_is_mujocos() {
    let cases = golden();
    assert_eq!(cases.len(), 37, "golden cases");
    let mut failures = Vec::new();
    for case in &cases {
        match (&case.outcome, load(case)) {
            (Outcome::Refused(words), Err(err)) => {
                if !err.to_string().contains(words.as_str()) {
                    failures.push(format!("{}: refused '{err}', MuJoCo '{words}'", case.name));
                }
            }
            (Outcome::Refused(words), Ok(_)) => {
                failures.push(format!("{}: loads, MuJoCo refuses '{words}'", case.name));
            }
            (Outcome::Loaded(_), Err(err)) => {
                failures.push(format!("{}: refused '{err}', MuJoCo loads it", case.name));
            }
            (Outcome::Loaded(meshes), Ok(model)) => {
                if model.mesh_data.len() != meshes.len() {
                    failures.push(format!(
                        "{}: {} meshes / {}",
                        case.name,
                        model.mesh_data.len(),
                        meshes.len()
                    ));
                    continue;
                }
                for (i, (want, data)) in meshes.iter().zip(&model.mesh_data).enumerate() {
                    if data.vertex_count() != want.vertices || data.triangle_count() != want.faces {
                        failures.push(format!(
                            "{} mesh {i}: {} vertices {} faces / {} {}",
                            case.name,
                            data.vertex_count(),
                            data.triangle_count(),
                            want.vertices,
                            want.faces
                        ));
                    }
                    if !case.files.is_empty() && data.hull().map(to_mujoco) != want.graph {
                        failures.push(format!("{} mesh {i}: graph differs", case.name));
                    }
                    match (data.octree().map(hashes), &want.octree) {
                        (None, None) => {}
                        (Some(ours), Some(theirs)) if same_octree(&ours, theirs) => {}
                        (ours, theirs) => failures.push(format!(
                            "{} mesh {i}: octree {ours:?} / {theirs:?}",
                            case.name
                        )),
                    }
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
