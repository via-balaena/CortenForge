//! Mesh file I/O for CortenForge.
//!
//! This crate provides loading and saving of triangle meshes in common formats:
//!
//! - **STL** (Stereolithography) - Binary and ASCII
//! - **OBJ** (Wavefront) - ASCII only
//! - **PLY** (Polygon File Format) - Binary and ASCII
//! - **3MF** (3D Manufacturing Format) - ZIP-based XML format
//! - **STEP** (CAD interchange) - Feature-gated (`step` feature)
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Other game engines
//! - Python bindings
//!
//! # Example
//!
//! ```no_run
//! use mesh_io::{load_stl, save_stl, MeshFormat};
//!
//! // Load a mesh
//! let mesh = load_stl("model.stl").unwrap();
//!
//! // Save it back
//! save_stl(&mesh, "output.stl", false).unwrap();
//! ```
//!
//! # Format Detection
//!
//! The crate can automatically detect file format from extension:
//!
//! ```no_run
//! use mesh_io::{load_mesh, save_mesh};
//!
//! // Format detected from .stl extension
//! let mesh = load_mesh("model.stl").unwrap();
//!
//! // Save to a different format
//! save_mesh(&mesh, "model.obj").unwrap();
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - >=90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod error;
mod obj;
mod ply;
#[cfg(feature = "step")]
mod step;
mod stl;
mod threemf;

pub use error::{IoError, IoResult};
pub use obj::{load_obj, save_obj};
pub use ply::{load_ply, save_ply};
#[cfg(feature = "step")]
pub use step::{load_step, save_step};
pub use stl::{load_stl, save_stl};
pub use threemf::{load_3mf, save_3mf};

use std::path::Path;

use mesh_types::IndexedMesh;

/// Supported mesh file formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MeshFormat {
    /// STL (Stereolithography) format.
    /// Supports binary and ASCII variants.
    Stl,
    /// OBJ (Wavefront) format.
    /// ASCII only, supports vertices and faces.
    Obj,
    /// PLY (Polygon File Format).
    /// Supports binary and ASCII variants.
    Ply,
    /// 3MF (3D Manufacturing Format).
    /// ZIP-based XML format for 3D printing.
    ThreeMf,
    /// STEP (Standard for the Exchange of Product Data).
    /// CAD interchange format (requires `step` feature).
    #[cfg(feature = "step")]
    Step,
}

impl MeshFormat {
    /// Detect format from file extension.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to check for extension
    ///
    /// # Returns
    ///
    /// The detected format, or `None` if the extension is not recognized.
    #[must_use]
    pub fn from_path<P: AsRef<Path>>(path: P) -> Option<Self> {
        let ext = path.as_ref().extension()?.to_str()?.to_lowercase();
        match ext.as_str() {
            "stl" => Some(Self::Stl),
            "obj" => Some(Self::Obj),
            "ply" => Some(Self::Ply),
            "3mf" => Some(Self::ThreeMf),
            #[cfg(feature = "step")]
            "step" | "stp" => Some(Self::Step),
            _ => None,
        }
    }

    /// Get the canonical file extension for this format.
    #[must_use]
    pub const fn extension(&self) -> &'static str {
        match self {
            Self::Stl => "stl",
            Self::Obj => "obj",
            Self::Ply => "ply",
            Self::ThreeMf => "3mf",
            #[cfg(feature = "step")]
            Self::Step => "step",
        }
    }
}

/// Load a mesh from a file, detecting format from extension.
///
/// # Arguments
///
/// * `path` - Path to the mesh file
///
/// # Errors
///
/// Returns an error if:
/// - The file format cannot be determined from the extension
/// - The file cannot be read
/// - The file content is invalid for the detected format
///
/// # Example
///
/// ```no_run
/// use mesh_io::load_mesh;
///
/// let mesh = load_mesh("model.stl").unwrap();
/// ```
pub fn load_mesh<P: AsRef<Path>>(path: P) -> IoResult<IndexedMesh> {
    let path = path.as_ref();
    let format = MeshFormat::from_path(path).ok_or_else(|| IoError::UnknownFormat {
        extension: path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("(none)")
            .to_string(),
    })?;

    match format {
        MeshFormat::Stl => load_stl(path),
        MeshFormat::Obj => load_obj(path),
        MeshFormat::Ply => load_ply(path),
        MeshFormat::ThreeMf => load_3mf(path),
        #[cfg(feature = "step")]
        MeshFormat::Step => load_step(path),
    }
}

/// Save a mesh to a file, detecting format from extension.
///
/// # Arguments
///
/// * `mesh` - The mesh to save
/// * `path` - Path for the output file
///
/// # Errors
///
/// Returns an error if:
/// - The file format cannot be determined from the extension
/// - The file cannot be written
///
/// # Example
///
/// ```no_run
/// use mesh_io::{save_mesh, load_mesh};
///
/// let mesh = load_mesh("input.stl").unwrap();
/// save_mesh(&mesh, "output.obj").unwrap();
/// ```
pub fn save_mesh<P: AsRef<Path>>(mesh: &IndexedMesh, path: P) -> IoResult<()> {
    let path = path.as_ref();
    let format = MeshFormat::from_path(path).ok_or_else(|| IoError::UnknownFormat {
        extension: path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("(none)")
            .to_string(),
    })?;

    match format {
        MeshFormat::Stl => save_stl(mesh, path, true), // Default to binary STL
        MeshFormat::Obj => save_obj(mesh, path),
        MeshFormat::Ply => save_ply(mesh, path, true), // Default to binary PLY
        MeshFormat::ThreeMf => save_3mf(mesh, path),
        #[cfg(feature = "step")]
        MeshFormat::Step => save_step(mesh, path),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_from_path_stl() {
        assert_eq!(MeshFormat::from_path("model.stl"), Some(MeshFormat::Stl));
        assert_eq!(MeshFormat::from_path("model.STL"), Some(MeshFormat::Stl));
        assert_eq!(
            MeshFormat::from_path("/path/to/model.stl"),
            Some(MeshFormat::Stl)
        );
    }

    #[test]
    fn format_from_path_obj() {
        assert_eq!(MeshFormat::from_path("model.obj"), Some(MeshFormat::Obj));
        assert_eq!(MeshFormat::from_path("model.OBJ"), Some(MeshFormat::Obj));
    }

    #[test]
    fn format_from_path_ply() {
        assert_eq!(MeshFormat::from_path("model.ply"), Some(MeshFormat::Ply));
        assert_eq!(MeshFormat::from_path("model.PLY"), Some(MeshFormat::Ply));
        assert_eq!(
            MeshFormat::from_path("/path/to/model.ply"),
            Some(MeshFormat::Ply)
        );
    }

    #[test]
    fn format_from_path_3mf() {
        assert_eq!(
            MeshFormat::from_path("model.3mf"),
            Some(MeshFormat::ThreeMf)
        );
        assert_eq!(
            MeshFormat::from_path("model.3MF"),
            Some(MeshFormat::ThreeMf)
        );
        assert_eq!(
            MeshFormat::from_path("/path/to/model.3mf"),
            Some(MeshFormat::ThreeMf)
        );
    }

    #[cfg(feature = "step")]
    #[test]
    fn format_from_path_step() {
        assert_eq!(MeshFormat::from_path("model.step"), Some(MeshFormat::Step));
        assert_eq!(MeshFormat::from_path("model.STEP"), Some(MeshFormat::Step));
        assert_eq!(MeshFormat::from_path("model.stp"), Some(MeshFormat::Step));
        assert_eq!(MeshFormat::from_path("model.STP"), Some(MeshFormat::Step));
        assert_eq!(
            MeshFormat::from_path("/path/to/model.step"),
            Some(MeshFormat::Step)
        );
    }

    #[test]
    fn format_from_path_unknown() {
        assert_eq!(MeshFormat::from_path("model.xyz"), None);
        assert_eq!(MeshFormat::from_path("model"), None);
        assert_eq!(MeshFormat::from_path(""), None);
    }

    #[test]
    fn format_extension() {
        assert_eq!(MeshFormat::Stl.extension(), "stl");
        assert_eq!(MeshFormat::Obj.extension(), "obj");
        assert_eq!(MeshFormat::Ply.extension(), "ply");
        assert_eq!(MeshFormat::ThreeMf.extension(), "3mf");
    }

    #[cfg(feature = "step")]
    #[test]
    fn format_extension_step() {
        assert_eq!(MeshFormat::Step.extension(), "step");
    }
}
