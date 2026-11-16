//! A mesh file read as MuJoCo 3.5.0 reads it.
//!
//! MuJoCo reads a mesh file by its format (`mjCMesh::LoadFromResource`,
//! `user_mesh.cc:646-666`) and hands every file's vertices to
//! `ProcessVertices` (`:543-614`), which refuses a coordinate that is not
//! finite and, for an STL, merges repeated vertices. An STL and an MSH are
//! read here as MuJoCo's `LoadSTL` and `LoadMSH` read them (`:1209-1277`,
//! `:1282-1368`); the other formats are read by mesh-io.

use nalgebra::{Point3, Vector3};
use std::cmp::Ordering;
use std::collections::HashMap;
use std::path::Path;

use super::asset::{AssetKind, resolve_asset_path};
use super::mesh::RawMesh;
use crate::error::{ElementKind, MjcfError, Result};
use crate::types::{MjcfCompiler, MjcfMesh};

/// A binary STL's 80-byte header and its 4-byte face count.
const STL_HEADER: usize = 84;

/// One face of a binary STL: its normal, three vertices and two attribute
/// bytes.
const STL_FACE: usize = 50;

/// The most faces MuJoCo 3.5.0 reads from an STL (`user_mesh.cc:1231`).
/// This crate reads more (book L36, Q118): MuJoCo's words are kept for a
/// file it refuses for its count that is not read here either.
const STL_MAX_FACES: u32 = 200_000;

/// The largest magnitude of an STL coordinate MuJoCo 3.5.0 reads, 2^30
/// (`user_mesh.cc:1259`).
const STL_MAX_COORD: f64 = 1_073_741_824.0;

/// A file's vertices, three floats each, and its faces' vertex ids, three
/// to a face, signed as MuJoCo 3.5.0 holds them (`face_`, ints): an MSH
/// file's may be negative, refused only after its vertices are checked
/// (`CheckInitialMesh`, `user_mesh.cc:1835-1841`).
type FileMesh = (Vec<f32>, Vec<i64>);

/// Load a mesh file's vertices and faces as MuJoCo 3.5.0 hands them to its
/// mesh compiler (`mjCMesh::TryCompile`, `user_mesh.cc:692-729`).
///
/// An STL or an MSH (by its extension, in any case) is read as MuJoCo's
/// `LoadSTL` or `LoadMSH` reads one ([`read_stl`], [`read_msh`]); OBJ, PLY
/// and, with the `threemf` feature, 3MF are read by mesh-io and rounded to
/// single precision, as MuJoCo's loaders hand a file's vertices on as
/// floats. Under a left-handed `scale` each
/// face is turned ([`turn_faces`]), as MuJoCo's loaders turn an STL's, an
/// OBJ's and an MSH's; then the vertices are checked, an STL's merged where
/// repeated ([`process_vertices`]).
///
/// # Errors
///
/// Returns an [`MjcfError::Asset`] naming the mesh if its path does not
/// resolve ([`resolve_asset_path`]), if the file cannot be read or is not
/// one of its format, in MuJoCo's words where MuJoCo reads the format, or
/// if it holds no vertex or no face, or a face naming a vertex it does not
/// hold.
pub fn load_mesh_file(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    mjcf_mesh: &MjcfMesh,
) -> Result<RawMesh> {
    let refuse = |reason: String| MjcfError::asset(ElementKind::Mesh, &mjcf_mesh.name, reason);
    let resolved_path =
        resolve_asset_path(file_path, base_path, compiler, AssetKind::Mesh).map_err(refuse)?;

    let read = |what: &str| {
        std::fs::read(&resolved_path)
            .map_err(|e| refuse(format!("could not read {what} file '{file_path}': {e}")))
    };
    let ((vert, mut faces), merge) = if has_extension(&resolved_path, "stl") {
        (read_stl(&read("STL")?, file_path).map_err(refuse)?, true)
    } else if has_extension(&resolved_path, "msh") {
        (read_msh(&read("MSH")?, file_path).map_err(refuse)?, false)
    } else {
        let indexed_mesh = mesh_io::load_mesh(&resolved_path)
            .map_err(|e| refuse(format!("failed to load '{file_path}': {e}")))?;
        let faces: Vec<i64> = indexed_mesh
            .faces
            .iter()
            .flatten()
            .map(|&i| i64::from(i))
            .collect();
        // MuJoCo 3.5.0's loaders hand a file's vertices to `ProcessVertices`
        // as floats (`user_mesh.cc:543`), so each is rounded to one, as an
        // embedded mesh's are when they are read.
        #[allow(clippy::cast_possible_truncation)] // the rounding is MuJoCo's
        let vert: Vec<f32> = indexed_mesh
            .vertices
            .iter()
            .flat_map(|v| [v.x as f32, v.y as f32, v.z as f32])
            .collect();
        ((vert, faces), false)
    };
    if left_handed(mjcf_mesh) {
        turn_faces(&mut faces);
    }
    let (vertices, indices) = process_vertices(&vert, faces, merge).map_err(refuse)?;

    if vertices.is_empty() {
        return Err(refuse(format!("file '{file_path}' contains no vertices")));
    }
    if indices.is_empty() {
        return Err(refuse(format!("file '{file_path}' contains no faces")));
    }
    // Each face's vertices exist (`CheckInitialMesh`, `user_mesh.cc:1835-
    // 1841`), in MuJoCo's words.
    let in_range = |index: &i64| usize::try_from(*index).is_ok_and(|i| i < vertices.len());
    if let Some(i) = indices.iter().position(|index| !in_range(index)) {
        return Err(refuse(format!(
            "in face {}, vertex index {} does not exist",
            i / 3,
            indices[i]
        )));
    }
    let indices = indices
        .into_iter()
        .filter_map(|i| usize::try_from(i).ok())
        .collect();
    Ok((vertices, indices))
}

/// Whether the file's extension is `ext`, in any case, as MuJoCo 3.5.0
/// tells a file's format by it (`mjuu_extToContentType`,
/// `user_util.cc:957-973`).
fn has_extension(path: &Path, ext: &str) -> bool {
    path.extension()
        .is_some_and(|e| e.eq_ignore_ascii_case(ext))
}

/// Whether a mesh's `scale` turns it inside out: its product is not
/// positive, as MuJoCo 3.5.0's file loaders test it (`righthand`,
/// `user_mesh.cc:1085`, `:1210`, `:1283`), so 0 and NaN count as left-handed.
fn left_handed(mjcf_mesh: &MjcfMesh) -> bool {
    let s = mjcf_mesh.scale.unwrap_or(Vector3::new(1.0, 1.0, 1.0));
    (s.x * s.y * s.z).partial_cmp(&0.0) != Some(Ordering::Greater)
}

/// Each face's second and third vertices exchanged, so a left-handed
/// `scale` leaves it facing out, as MuJoCo 3.5.0's `LoadSTL`, `LoadOBJ` and
/// `LoadMSH` place them (`user_mesh.cc:1263-1268`, `:1097-1104`,
/// `:1359-1366`).
fn turn_faces(faces: &mut [i64]) {
    for face in faces.chunks_exact_mut(3) {
        face.swap(1, 2);
    }
}

/// An STL's vertices (three floats each, three vertices per face, in the
/// file's order) and its faces' vertex ids, as MuJoCo 3.5.0's `LoadSTL`
/// reads them (`user_mesh.cc:1209-1277`), before [`process_vertices`].
///
/// A file exactly the size its face count gives, 84 bytes and 50 per face,
/// is binary; one that is not, begins with `solid` and is text (UTF-8,
/// with no NUL byte) is read as ASCII ([`read_ascii_stl`]), which MuJoCo
/// does not read (book L36, Q118). A
/// binary file of more than 200,000 faces is read too (Q118). Otherwise the
/// file is refused as MuJoCo refuses it, in its order and words: empty,
/// shorter than its header, a count of 0 or (where the size does not fit
/// either) above 200,000, the wrong size. Each vertex is refused when a
/// coordinate's magnitude is above 2^30 (Q119); a NaN passes that test, as
/// it does MuJoCo's, and is refused by [`process_vertices`].
///
/// # Errors
///
/// Returns the reason as MuJoCo words it, naming the file as written
/// (`file`).
fn read_stl(bytes: &[u8], file: &str) -> std::result::Result<FileMesh, String> {
    if bytes.is_empty() {
        return Err(format!("STL file '{file}' is empty"));
    }
    let count = bytes
        .get(STL_HEADER - 4..STL_HEADER)
        .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]));
    let binary = count
        .is_some_and(|n| bytes.len() as u64 == STL_HEADER as u64 + STL_FACE as u64 * u64::from(n));
    if !binary
        && starts_with_solid(bytes)
        && !bytes.contains(&0)
        && let Ok(text) = std::str::from_utf8(bytes)
    {
        return read_ascii_stl(text, file);
    }
    let Some(n) = count else {
        return Err(format!("invalid header in STL file '{file}'"));
    };
    if n < 1 || (n > STL_MAX_FACES && !binary) {
        return Err(format!(
            "number of faces should be between 1 and 200000 in STL file '{file}'; perhaps this \
             is an ASCII file?"
        ));
    }
    if !binary {
        return Err(format!(
            "STL file '{file}' has wrong size; perhaps this is an ASCII file?"
        ));
    }
    let n = n as usize;
    let vertices = (0..3 * n).map(|k| {
        let at = STL_HEADER + STL_FACE * (k / 3) + 12 * (k % 3 + 1);
        std::array::from_fn(|c| {
            let b = &bytes[at + 4 * c..at + 4 * c + 4];
            f32::from_le_bytes([b[0], b[1], b[2], b[3]])
        })
    });
    stl_soup(vertices, file)
}

/// An STL's vertices, three to a face in the file's order, as MuJoCo
/// 3.5.0's `LoadSTL` gathers them (`user_mesh.cc:1252-1274`): one whose
/// coordinate is above 2^30 in magnitude refused, each vertex its own id.
fn stl_soup(
    vertices: impl Iterator<Item = [f32; 3]>,
    file: &str,
) -> std::result::Result<FileMesh, String> {
    let mut vert = Vec::new();
    for v in vertices {
        if v.iter().any(|&c| f64::from(c.abs()) > STL_MAX_COORD) {
            return Err(format!("vertex in STL file '{file}' exceed maximum bounds"));
        }
        vert.extend_from_slice(&v);
    }
    let faces = (0..).take(vert.len() / 3).collect();
    Ok((vert, faces))
}

/// Whether a file begins, past any whitespace, with `solid`, the word an
/// ASCII STL begins with.
fn starts_with_solid(bytes: &[u8]) -> bool {
    let start = bytes
        .iter()
        .position(|b| !b.is_ascii_whitespace())
        .unwrap_or(bytes.len());
    bytes[start..].starts_with(b"solid")
}

/// An ASCII STL read as [`read_stl`] reads a binary one, which MuJoCo
/// 3.5.0 does not do (book L36, Q118: lenient): each `vertex` line's three
/// numbers are parsed straight to single precision, three lines to a face in
/// the file's order, gathered as [`stl_soup`] gathers a binary file's.
///
/// # Errors
///
/// Returns why, naming the file and the line counted from 1, where a
/// `vertex` line does not hold three numbers or the file's vertices do not
/// make whole faces.
fn read_ascii_stl(text: &str, file: &str) -> std::result::Result<FileMesh, String> {
    let mut read = Vec::new();
    for (line, l) in text.lines().enumerate() {
        let mut words = l.split_whitespace();
        if !words
            .next()
            .is_some_and(|w| w.eq_ignore_ascii_case("vertex"))
        {
            continue;
        }
        let numbers: Vec<f32> = words
            .map(str::parse::<f32>)
            .collect::<std::result::Result<_, _>>()
            .map_err(|e| format!("in ASCII STL file '{file}', line {}: {e}", line + 1))?;
        let [x, y, z] = numbers[..] else {
            return Err(format!(
                "in ASCII STL file '{file}', line {}: a vertex has {} numbers, not 3",
                line + 1,
                numbers.len()
            ));
        };
        read.push([x, y, z]);
    }
    if read.len() % 3 != 0 {
        return Err(format!(
            "ASCII STL file '{file}' holds {} vertices, not three to a face",
            read.len()
        ));
    }
    stl_soup(read.into_iter(), file)
}

/// An MSH file's vertices and faces, as MuJoCo 3.5.0's `LoadMSH` reads
/// them (`user_mesh.cc:1282-1368`): four 32-bit counts (vertices, normals,
/// texture coordinates, faces), then the vertices' floats, the normals' and
/// the texture coordinates' (not read here), and the faces' vertex ids.
///
/// # Errors
///
/// Returns MuJoCo's words, naming the file as written (`file`), in its
/// order: an empty file; one shorter than its counts; fewer than 4
/// vertices, a negative count, or normals or texture coordinates that are
/// not one per vertex; a count whose bytes would pass `INT_MAX`; a file
/// not exactly the size its counts give.
fn read_msh(bytes: &[u8], file: &str) -> std::result::Result<FileMesh, String> {
    if bytes.is_empty() {
        return Err(format!("MSH file '{file}' is empty"));
    }
    if bytes.len() < 16 {
        return Err(format!("missing header in MSH file '{file}'"));
    }
    let word = |at: usize| [bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]];
    let int = |at: usize| i32::from_le_bytes(word(at));
    let [nv, nn, nt, nf] = [int(0), int(4), int(8), int(12)].map(i64::from);
    if nv < 4 || nf < 0 || nn < 0 || nt < 0 || (nn > 0 && nn != nv) || (nt > 0 && nt != nv) {
        return Err(format!("invalid sizes in MSH file '{file}'"));
    }
    let below = |n: i64, floats: i64| n < i64::from(i32::MAX) / 4 / floats;
    if !(below(nv, 3) && below(nn, 3) && below(nt, 2) && below(nf, 3)) {
        return Err(format!("too large sizes in MSH file '{file}'."));
    }
    let faces_at = 16 + 4 * (3 * nv + 3 * nn + 2 * nt);
    if i64::try_from(bytes.len()) != Ok(faces_at + 12 * nf) {
        return Err(format!("unexpected file size in MSH file '{file}'"));
    }
    // Every count is checked non-negative and bounded above.
    let [nv, nf, faces_at] = [nv, nf, faces_at].map(|n| usize::try_from(n).unwrap_or_default());
    let vert = (0..3 * nv)
        .map(|k| f32::from_le_bytes(word(16 + 4 * k)))
        .collect();
    let faces = (0..3 * nf)
        .map(|k| i64::from(int(faces_at + 4 * k)))
        .collect();
    Ok((vert, faces))
}

/// A file's vertices (`vert`, three floats each) and its faces' vertex ids,
/// as MuJoCo 3.5.0's `ProcessVertices` passes them on
/// (`user_mesh.cc:543-614`): each in double precision, a coordinate that is
/// not finite refused. Where `merge` (an STL: `remove_repeated`, `:1276`), vertices
/// equal as floats, `-0` and `0` among them, are made one: numbered in the
/// order they first appear, each keeping its first appearance's value, the
/// faces renumbered.
///
/// # Errors
///
/// Returns MuJoCo's words for the first coordinate that is not finite,
/// naming its vertex's index where it does not merge and the index of the
/// vertex's first float where it does, as MuJoCo's two loops count
/// (`:557-561`, `:573-577`).
fn process_vertices(
    vert: &[f32],
    mut faces: Vec<i64>,
    merge: bool,
) -> std::result::Result<(Vec<Point3<f64>>, Vec<i64>), String> {
    let point = |c: &[f32]| Point3::new(f64::from(c[0]), f64::from(c[1]), f64::from(c[2]));
    let not_finite = |c: &[f32]| !c.iter().all(|x| x.is_finite());
    if !merge {
        if let Some(i) = vert.chunks_exact(3).position(not_finite) {
            return Err(format!("vertex coordinate {i} is not finite"));
        }
        return Ok((vert.chunks_exact(3).map(point).collect(), faces));
    }
    // Float `==`: a key's bits, with -0 read as 0 (a NaN is refused first).
    let key = |c: &[f32]| -> [u32; 3] {
        std::array::from_fn(|k| if c[k] == 0.0 { 0 } else { c[k].to_bits() })
    };
    let mut ids: HashMap<[u32; 3], i64> = HashMap::with_capacity(vert.len() / 3);
    let mut kept: Vec<Point3<f64>> = Vec::new();
    let mut renumber = Vec::with_capacity(vert.len() / 3);
    let mut count: i64 = 0;
    for (i, c) in vert.chunks_exact(3).enumerate() {
        if not_finite(c) {
            return Err(format!("vertex coordinate {} is not finite", 3 * i));
        }
        let id = *ids.entry(key(c)).or_insert_with(|| {
            kept.push(point(c));
            count += 1;
            count - 1
        });
        renumber.push(id);
    }
    for f in &mut faces {
        // An STL's faces name its vertices in the order read ([`stl_soup`]).
        if let Some(&id) = usize::try_from(*f).ok().and_then(|k| renumber.get(k)) {
            *f = id;
        }
    }
    Ok((kept, faces))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::{load_mesh_file, process_vertices};
    use crate::types::{MjcfCompiler, MjcfMesh};

    /// A mesh of no attributes but its name.
    fn named(name: &str) -> MjcfMesh {
        MjcfMesh {
            name: name.to_string(),
            ..MjcfMesh::default()
        }
    }

    /// A tetrahedron saved by mesh-io as a binary STL (each face's three
    /// vertices its own).
    fn create_test_stl(path: &std::path::Path) {
        use mesh_types::{IndexedMesh, Point3};

        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);

        mesh_io::save_mesh(&mesh, path).expect("Failed to save test STL");
    }

    /// An STL's four faces load, their twelve vertices made four.
    #[test]
    fn test_load_mesh_file_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        create_test_stl(&mesh_path);

        let (vertices, indices) = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            &named("test_mesh"),
        )
        .unwrap();
        assert_eq!(vertices.len(), 4);
        assert_eq!(indices.len(), 12);
    }

    /// Of vertices equal as floats the first is kept, as MuJoCo 3.5.0's map
    /// keeps the key it first inserts (`user_mesh.cc:580-584`, `:603-613`):
    /// `-0` first stays `-0`, `0` first stays `0`.
    #[test]
    fn process_vertices_keeps_the_first_of_equal_vertices() {
        for (first, second) in [(-0.0f32, 0.0f32), (0.0, -0.0)] {
            let vert = [first, 1.0, 2.0, second, 1.0, 2.0, 3.0, 4.0, 5.0];
            let (kept, faces) = process_vertices(&vert, vec![0, 1, 2], true).unwrap();
            assert_eq!(faces, vec![0, 0, 1]);
            assert_eq!(kept.len(), 2);
            assert_eq!(kept[0].x.to_bits(), f64::from(first).to_bits());
        }
    }

    /// A file of no format mesh-io reads is refused.
    #[test]
    fn test_load_mesh_file_unsupported_format() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.xyz");
        std::fs::write(&mesh_path, b"invalid mesh data").unwrap();

        let result = load_mesh_file(
            &mesh_path.to_string_lossy(),
            None,
            &MjcfCompiler::default(),
            &named("unsupported"),
        );

        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("failed to load"));
    }

    /// A file named `.stl` shorter than an STL's header is refused in
    /// MuJoCo 3.5.0's words, naming the mesh, not a panic.
    #[test]
    fn test_load_mesh_file_corrupt_stl() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("corrupt.stl");
        std::fs::write(&mesh_path, b"this is not a valid stl file content").unwrap();
        let file = mesh_path.to_string_lossy().to_string();

        let err = load_mesh_file(
            &file,
            None,
            &MjcfCompiler::default(),
            &named("corrupt_mesh"),
        )
        .unwrap_err()
        .to_string();
        assert_eq!(
            err,
            format!("mesh 'corrupt_mesh': invalid header in STL file '{file}'")
        );
    }
}
