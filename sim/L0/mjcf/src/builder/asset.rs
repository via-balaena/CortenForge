//! Asset path resolution for MJCF mesh, hfield, and texture files.
//!
//! Implements MuJoCo's path resolution rules: `strippath`, `assetdir`,
//! `meshdir`, and `texturedir` compiler settings are applied in the correct
//! priority order to locate asset files on disk.

use std::path::{Path, PathBuf};

use super::ModelConversionError;
use crate::types::MjcfCompiler;

/// Asset type for path resolution.
#[allow(dead_code)] // Texture used when texture loading lands
pub enum AssetKind {
    /// Mesh or hfield file (uses `meshdir`).
    Mesh,
    /// Texture file (uses `texturedir`).
    Texture,
}

/// Resolve an asset file path using compiler path settings.
///
/// # Path Resolution Rules (matches MuJoCo exactly)
///
/// 1. If `strippath` is enabled, strip directory components from `file_path`
/// 2. If the result is absolute, use it directly
/// 3. Otherwise, try type-specific dir (`meshdir`/`texturedir`), then `assetdir`,
///    then bare `base_path`
///
/// # Errors
///
/// Returns `ModelConversionError` if:
/// - Path is relative but no base path or directory is available
/// - Resolved path does not exist
pub fn resolve_asset_path(
    file_path: &str,
    base_path: Option<&Path>,
    compiler: &MjcfCompiler,
    kind: AssetKind,
) -> std::result::Result<PathBuf, ModelConversionError> {
    // A9. strippath: strip directory components, keeping only the base filename.
    let file_name = if compiler.strippath {
        Path::new(file_path)
            .file_name()
            .map(|f| f.to_string_lossy().to_string())
            .unwrap_or_else(|| file_path.to_string())
    } else {
        file_path.to_string()
    };

    let path = Path::new(&file_name);

    // Absolute paths are used directly
    if path.is_absolute() {
        if !path.exists() {
            return Err(ModelConversionError {
                message: format!("asset file not found: '{}'", path.display()),
            });
        }
        return Ok(path.to_path_buf());
    }

    // A3. Type-specific directory takes precedence over assetdir.
    let type_dir = match kind {
        AssetKind::Mesh => compiler.meshdir.as_deref(),
        AssetKind::Texture => compiler.texturedir.as_deref(),
    };

    // Try: type-specific dir, then assetdir, then bare base_path
    let resolved = if let Some(dir) = type_dir.or(compiler.assetdir.as_deref()) {
        let dir_path = Path::new(dir);
        if dir_path.is_absolute() {
            dir_path.join(path)
        } else if let Some(base) = base_path {
            base.join(dir_path).join(path)
        } else {
            // No base path but relative dir — try dir/filename as-is
            dir_path.join(path)
        }
    } else if let Some(base) = base_path {
        base.join(path)
    } else {
        return Err(ModelConversionError {
            message: format!(
                "asset file '{file_path}' is relative but no base path provided (use load_model_from_file or pass base_path to model_from_mjcf)"
            ),
        });
    };

    if !resolved.exists() {
        return Err(ModelConversionError {
            message: format!(
                "asset file not found: '{}' (resolved to '{}')",
                file_path,
                resolved.display()
            ),
        });
    }

    Ok(resolved)
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
            result.unwrap_err().to_string().contains("not found"),
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
        let err = result.unwrap_err().to_string();
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
        assert!(result.unwrap_err().to_string().contains("not found"));
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
}
