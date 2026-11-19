//! Asset path resolution for MJCF mesh, hfield, and texture files.
//!
//! An asset's path is built as MuJoCo 3.5.0 builds it, as text: `strippath`,
//! `meshdir` or `texturedir` (else `assetdir`) and the model's directory
//! applied in its order, `/` and `\` both separators, `.` and `..` reduced
//! by the text ([`resolve_asset_path`]).

use std::path::{Path, PathBuf};

use crate::types::MjcfCompiler;

/// Asset type for path resolution.
#[derive(Clone, Copy)]
pub enum AssetKind {
    /// Mesh or hfield file (uses `meshdir`).
    Mesh,
    /// Texture file (uses `texturedir`).
    /// Constructed only in tests; production texture loading is deferred.
    #[allow(dead_code)]
    Texture,
}

/// Resolve an asset file path as MuJoCo 3.5.0 resolves it.
///
/// MuJoCo reduces every path it reads (`FilePath`, `PathReduce`,
/// `user_util.cc:1006-1035`; [`path_reduce`]): `/` and `\` both separate, on
/// every platform. The `file` is reduced as it is read (`ReadAttrFile`,
/// `xml_util.cc:760-768`); under `strippath` only its part past the last
/// separator is kept (`mjuu_strippath`, `user_util.cc:768-781`); the type's
/// directory (`meshdir` or `texturedir`, else `assetdir`,
/// `xml_native_reader.cc:1124-1135`) is put before it
/// unless it is absolute ([`combine`]), and the two reduced together
/// (`mjCMesh::TryCompile`, `user_mesh.cc:706-719`); and a path still
/// relative is opened from the model's directory, the two reduced together
/// again (`VFS::Open`, `user_vfs.cc:151-152`). So `..` is taken away by its text, not
/// by the file system: `sub/nowhere/../m.obj` is `sub/m.obj` whether or not
/// `sub/nowhere` exists.
///
/// # Errors
///
/// Returns why, for the caller to name its asset, if:
/// - Path is relative but no base path or directory is available
/// - Resolved path does not exist
pub fn resolve_asset_path(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    kind: AssetKind,
) -> Result<PathBuf, String> {
    let path = in_type_dir(&stripped_file(file_path, compiler), compiler, kind);
    let resolved = if !abs_prefix(&path).is_empty() {
        PathBuf::from(&path)
    } else if let Some(base) = base_path {
        match base.to_str() {
            Some(base) => PathBuf::from(path_reduce(&combine(base, &path))),
            None => base.join(&path),
        }
    } else if type_dir(compiler, kind).is_some() {
        // No model directory: the directory and file, relative to the
        // process's working directory, as before.
        PathBuf::from(&path)
    } else {
        return Err(format!(
            "asset file '{file_path}' is relative but no base path provided (use load_model_from_file or pass base_path to model_from_mjcf)"
        ));
    };

    if !resolved.exists() {
        return Err(format!(
            "asset file not found: '{}' (resolved to '{}')",
            file_path,
            resolved.display()
        ));
    }
    Ok(resolved)
}

/// An asset's `file` as MuJoCo 3.5.0 holds it once read: reduced
/// ([`path_reduce`]), and under `strippath` only its part past the last
/// separator, which may be nothing (`mjuu_strippath`, `user_util.cc:768-781`).
pub fn stripped_file(file_path: &str, compiler: &MjcfCompiler) -> String {
    let file = path_reduce(file_path);
    match file.rfind(is_separator) {
        Some(cut) if compiler.strippath => file[cut + 1..].to_string(),
        _ => file,
    }
}

/// `file`, as [`stripped_file`] gives it, put after the type's directory
/// unless it is absolute ([`combine`]), the two reduced together: the path
/// MuJoCo 3.5.0 opens from the model's directory.
pub fn in_type_dir(file: &str, compiler: &MjcfCompiler, kind: AssetKind) -> String {
    match type_dir(compiler, kind) {
        Some(dir) => path_reduce(&combine(&path_reduce(dir), file)),
        None => file.to_string(),
    }
}

/// The directory an asset of `kind` is read from: `meshdir` or
/// `texturedir`, else `assetdir`.
fn type_dir(compiler: &MjcfCompiler, kind: AssetKind) -> Option<&str> {
    match kind {
        AssetKind::Mesh => compiler.meshdir.as_deref(),
        AssetKind::Texture => compiler.texturedir.as_deref(),
    }
    .or(compiler.assetdir.as_deref())
}

/// Whether `c` separates a path's parts for MuJoCo 3.5.0: `/` or `\`
/// (`FilePath::IsSeparator`, `user_util.h:220`).
fn is_separator(c: char) -> bool {
    c == '/' || c == '\\'
}

/// The part of `path` that makes it absolute for MuJoCo 3.5.0, or `""`
/// (`FilePath::AbsPrefix`, `user_util.cc:1063-1094`): a leading separator,
/// or everything to a `:/` or `:\` (a drive, or a scheme such as
/// `file:/`). MuJoCo's resource providers' own prefixes are not read here:
/// sim-mjcf registers none.
fn abs_prefix(path: &str) -> &str {
    if path.starts_with(is_separator) {
        return &path[..1];
    }
    match path.find(":/").or_else(|| path.find(":\\")) {
        Some(at) => &path[..at + 2],
        None => "",
    }
}

/// `first` and `second` joined as MuJoCo 3.5.0's `FilePath::Combine` joins
/// them (`user_util.cc:993-1004`): `second` alone where it is absolute,
/// else the two with a `/` between unless `first` is empty or ends in a
/// separator.
fn combine(first: &str, second: &str) -> String {
    if !abs_prefix(second).is_empty() {
        return second.to_string();
    }
    if !first.is_empty() && !first.ends_with(is_separator) {
        format!("{first}/{second}")
    } else {
        format!("{first}{second}")
    }
}

/// `path` reduced as MuJoCo 3.5.0's `FilePath::PathReduce` reduces it
/// (`user_util.cc:1006-1035`): past its absolute prefix ([`abs_prefix`]),
/// split at every `/` and `\`; a `.` part dropped; a `..` part taking the
/// part before it away, unless there is none or that part is `..` itself;
/// the parts, empty ones among them, joined by `/`.
fn path_reduce(path: &str) -> String {
    let prefix = abs_prefix(path);
    let mut parts: Vec<&str> = Vec::new();
    let mut rest = &path[prefix.len()..];
    while let Some(at) = rest.find(is_separator) {
        let part = &rest[..at];
        rest = &rest[at + 1..];
        if part == ".." && parts.last().is_some_and(|&last| last != "..") {
            parts.pop();
        } else if part != "." {
            parts.push(part);
        }
    }
    parts.push(rest);
    format!("{prefix}{}", parts.join("/"))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    /// Test resolve_asset_path with absolute path.
    #[test]
    fn test_resolve_asset_path_absolute() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let abs_path = mesh_path.to_string_lossy().to_string();
        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(&abs_path, None, &compiler, AssetKind::Mesh);
        assert!(
            result.is_ok(),
            "Absolute path should resolve without base_path"
        );
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Test resolve_asset_path fails for non-existent absolute path.
    #[test]
    fn test_resolve_asset_path_absolute_not_found() {
        // Use platform-appropriate absolute path that definitely doesn't exist
        #[cfg(windows)]
        let nonexistent_path = "C:\\nonexistent_dir_12345\\mesh.stl";
        #[cfg(not(windows))]
        let nonexistent_path = "/nonexistent_dir_12345/mesh.stl";

        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(nonexistent_path, None, &compiler, AssetKind::Mesh);
        assert!(result.is_err());
        assert!(
            result.unwrap_err().contains("not found"),
            "Error should mention 'not found' for non-existent absolute path"
        );
    }

    /// Test resolve_asset_path with relative path and base_path.
    #[test]
    fn test_resolve_asset_path_relative() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("meshes").join("test.stl");
        std::fs::create_dir_all(mesh_path.parent().unwrap()).unwrap();
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(
            "meshes/test.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Test resolve_asset_path fails for relative path without base_path.
    #[test]
    fn test_resolve_asset_path_relative_no_base() {
        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path("meshes/test.stl", None, &compiler, AssetKind::Mesh);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.contains("relative"));
        assert!(err.contains("no base path"));
    }

    /// Test resolve_asset_path fails for relative path when file doesn't exist.
    #[test]
    fn test_resolve_asset_path_relative_not_found() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let compiler = MjcfCompiler::default();
        let result = resolve_asset_path(
            "nonexistent.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("not found"));
    }

    /// Test resolve_asset_path with meshdir setting.
    #[test]
    fn test_resolve_asset_path_meshdir() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("custom_meshes").join("test.stl");
        std::fs::create_dir_all(mesh_path.parent().unwrap()).unwrap();
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let mut compiler = MjcfCompiler::default();
        compiler.meshdir = Some("custom_meshes".to_string());
        let result = resolve_asset_path(
            "test.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_ok(), "meshdir should resolve: {result:?}");
        assert_eq!(result.unwrap(), mesh_path);
    }

    /// Test resolve_asset_path with strippath.
    #[test]
    fn test_resolve_asset_path_strippath() {
        let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
        let mesh_path = temp_dir.path().join("test.stl");
        std::fs::write(&mesh_path, b"dummy").expect("Failed to write test file");

        let mut compiler = MjcfCompiler::default();
        compiler.strippath = true;
        // File path has directory components, but strippath removes them
        let result = resolve_asset_path(
            "some/nested/dir/test.stl",
            Some(temp_dir.path()),
            &compiler,
            AssetKind::Mesh,
        );
        assert!(result.is_ok(), "strippath should strip dirs: {result:?}");
        assert_eq!(result.unwrap(), mesh_path);
    }

    #[test]
    fn test_resolve_asset_path_texturedir() {
        let dir = tempfile::tempdir().unwrap();
        let tex_dir = dir.path().join("textures");
        std::fs::create_dir_all(&tex_dir).unwrap();
        std::fs::write(tex_dir.join("wood.png"), b"fake_texture").unwrap();

        let mut compiler = MjcfCompiler::default();
        compiler.texturedir = Some("textures/".to_string());

        let result =
            resolve_asset_path("wood.png", Some(dir.path()), &compiler, AssetKind::Texture);
        assert!(
            result.is_ok(),
            "texturedir should resolve texture paths: {result:?}"
        );
        assert!(
            result.unwrap().ends_with("textures/wood.png"),
            "should resolve to textures/wood.png"
        );
    }

    #[test]
    fn test_resolve_asset_path_texturedir_vs_meshdir() {
        // meshdir applies to meshes, texturedir applies to textures
        let dir = tempfile::tempdir().unwrap();
        let mesh_dir = dir.path().join("meshes");
        let tex_dir = dir.path().join("textures");
        std::fs::create_dir_all(&mesh_dir).unwrap();
        std::fs::create_dir_all(&tex_dir).unwrap();
        std::fs::write(mesh_dir.join("box.stl"), b"fake_mesh").unwrap();
        std::fs::write(tex_dir.join("wood.png"), b"fake_texture").unwrap();

        let mut compiler = MjcfCompiler::default();
        compiler.meshdir = Some("meshes/".to_string());
        compiler.texturedir = Some("textures/".to_string());

        let mesh_result =
            resolve_asset_path("box.stl", Some(dir.path()), &compiler, AssetKind::Mesh);
        let tex_result =
            resolve_asset_path("wood.png", Some(dir.path()), &compiler, AssetKind::Texture);
        assert!(mesh_result.is_ok(), "mesh should resolve via meshdir");
        assert!(tex_result.is_ok(), "texture should resolve via texturedir");
        assert!(
            mesh_result.unwrap().to_string_lossy().contains("meshes"),
            "mesh path should use meshdir"
        );
        assert!(
            tex_result.unwrap().to_string_lossy().contains("textures"),
            "texture path should use texturedir"
        );
    }

    /// A path reduced as MuJoCo 3.5.0's `FilePath::PathReduce` reduces it
    /// (`user_util.cc:1006-1035`, its steps traced by hand): `/` and `\` both
    /// split it, `.` goes, `..` takes away the part before it unless there is
    /// none or that part is `..`, empty parts stay, and the parts are joined
    /// by `/` after the absolute prefix as written.
    #[test]
    fn a_path_is_reduced_as_mujoco_reduces_it() {
        for (path, reduced) in [
            ("sub/./c.obj", "sub/c.obj"),
            (r"sub\c.obj", "sub/c.obj"),
            ("sub/nowhere/../c.obj", "sub/c.obj"),
            ("./c.obj", "c.obj"),
            ("../c.obj", "../c.obj"),
            ("../../c.obj", "../../c.obj"),
            ("a/../../c.obj", "../c.obj"),
            ("sub//c.obj", "sub//c.obj"),
            ("sub/", "sub/"),
            ("/abs/./x/../c.obj", "/abs/c.obj"),
            (r"\abs\c.obj", r"\abs/c.obj"),
            (r"C:\dir\..\c.obj", r"C:\c.obj"),
            ("file:/a/../b", "file:/b"),
        ] {
            assert_eq!(path_reduce(path), reduced, "{path}");
        }
    }

    /// The absolute prefix and the join, as MuJoCo 3.5.0's
    /// `FilePath::AbsPrefix` (`user_util.cc:1063-1094`) and
    /// `FilePath::Combine` (`:993-1004`) make them.
    #[test]
    fn a_path_is_absolute_and_joined_as_mujoco_takes_it() {
        for (path, prefix) in [
            ("", ""),
            ("rel/c.obj", ""),
            ("/x", "/"),
            (r"\x", r"\"),
            ("C:/x", "C:/"),
            (r"C:\x", r"C:\"),
            (r"a:\b:/c", "a:\\b:/"),
        ] {
            assert_eq!(abs_prefix(path), prefix, "{path}");
        }
        for (first, second, joined) in [
            ("dir", "c.obj", "dir/c.obj"),
            ("dir/", "c.obj", "dir/c.obj"),
            (r"dir\", "c.obj", r"dir\c.obj"),
            ("", "c.obj", "c.obj"),
            ("dir", "/abs/c.obj", "/abs/c.obj"),
            ("dir", r"C:\c.obj", r"C:\c.obj"),
        ] {
            assert_eq!(combine(first, second), joined, "{first} {second}");
        }
    }
}
