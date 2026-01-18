//! Export functionality for assemblies.
//!
//! Supports exporting to various formats including 3MF and STL.

use std::path::Path;

use crate::assembly::Assembly;
use crate::error::{AssemblyError, AssemblyResult};

/// Assembly export format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssemblyExportFormat {
    /// 3MF with multiple objects and build items.
    ThreeMf,
    /// Single merged STL file.
    StlMerged,
    /// Separate STL files for each part.
    StlSeparate,
}

impl AssemblyExportFormat {
    /// Determine format from file extension.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_assembly::AssemblyExportFormat;
    /// use std::path::Path;
    ///
    /// assert_eq!(
    ///     AssemblyExportFormat::from_path(Path::new("output.3mf")),
    ///     Some(AssemblyExportFormat::ThreeMf)
    /// );
    /// assert_eq!(
    ///     AssemblyExportFormat::from_path(Path::new("output.stl")),
    ///     Some(AssemblyExportFormat::StlMerged)
    /// );
    /// ```
    #[must_use]
    pub fn from_path(path: &Path) -> Option<Self> {
        let ext = path.extension()?.to_str()?.to_lowercase();
        match ext.as_str() {
            "3mf" => Some(Self::ThreeMf),
            "stl" => Some(Self::StlMerged),
            _ => None,
        }
    }

    /// Get the file extension for this format.
    #[must_use]
    pub fn extension(&self) -> &'static str {
        match self {
            Self::ThreeMf => "3mf",
            Self::StlMerged | Self::StlSeparate => "stl",
        }
    }
}

/// Save the assembly to a file.
///
/// The format is determined by the file extension or explicit format option.
///
/// # Supported Formats
/// - `.3mf` - 3MF with multiple objects and build items (requires `export-3mf` feature)
/// - `.stl` - Merged mesh as single STL
///
/// # Errors
///
/// Returns an error if:
/// - The assembly is empty
/// - The file cannot be written
/// - 3MF export is requested but the feature is not enabled
pub fn save_assembly(
    assembly: &Assembly,
    path: &Path,
    format: Option<AssemblyExportFormat>,
) -> AssemblyResult<()> {
    if assembly.is_empty() {
        return Err(AssemblyError::EmptyAssembly);
    }

    let format = format.unwrap_or_else(|| {
        AssemblyExportFormat::from_path(path).unwrap_or(AssemblyExportFormat::ThreeMf)
    });

    match format {
        AssemblyExportFormat::ThreeMf => save_3mf(assembly, path),
        AssemblyExportFormat::StlMerged => save_stl_merged(assembly, path),
        AssemblyExportFormat::StlSeparate => save_stl_separate(assembly, path),
    }
}

/// Save the assembly as a merged STL file.
fn save_stl_merged(assembly: &Assembly, path: &Path) -> AssemblyResult<()> {
    let merged = assembly.to_merged_mesh();
    mesh_io::save_stl(&merged, path, true)?;
    Ok(())
}

/// Save each part as a separate STL file.
fn save_stl_separate(assembly: &Assembly, path: &Path) -> AssemblyResult<()> {
    let parent = path.parent().unwrap_or(Path::new("."));
    let stem = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("assembly");

    for part in assembly.parts() {
        // Skip invisible parts
        if !part.is_visible() {
            continue;
        }

        // Get transformed mesh
        let mesh = assembly
            .get_transformed_mesh(part.id())
            .unwrap_or_else(|| part.mesh().clone());

        // Create filename
        let filename = format!("{}_{}.stl", stem, sanitize_filename(part.id()));
        let file_path = parent.join(filename);

        mesh_io::save_stl(&mesh, &file_path, true)?;
    }

    Ok(())
}

/// Save the assembly as a 3MF file.
#[cfg(feature = "export-3mf")]
fn save_3mf(assembly: &Assembly, path: &Path) -> AssemblyResult<()> {
    use std::fs::File;
    use std::io::Write;
    use zip::write::SimpleFileOptions;
    use zip::ZipWriter;

    let file = File::create(path).map_err(|e| AssemblyError::Io {
        path: path.to_path_buf(),
        source: e,
    })?;

    let mut zip = ZipWriter::new(file);
    let options = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    // Write content types file
    zip.start_file("[Content_Types].xml", options)
        .map_err(|e| AssemblyError::Zip {
            message: e.to_string(),
        })?;
    zip.write_all(CONTENT_TYPES_XML.as_bytes())
        .map_err(|e| AssemblyError::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

    // Write relationships file
    zip.start_file("_rels/.rels", options)
        .map_err(|e| AssemblyError::Zip {
            message: e.to_string(),
        })?;
    zip.write_all(RELS_XML.as_bytes())
        .map_err(|e| AssemblyError::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

    // Write the model file
    zip.start_file("3D/3dmodel.model", options)
        .map_err(|e| AssemblyError::Zip {
            message: e.to_string(),
        })?;

    let model_xml = generate_3mf_model_xml(assembly);
    zip.write_all(model_xml.as_bytes())
        .map_err(|e| AssemblyError::Io {
            path: path.to_path_buf(),
            source: e,
        })?;

    zip.finish().map_err(|e| AssemblyError::Zip {
        message: e.to_string(),
    })?;

    Ok(())
}

#[cfg(not(feature = "export-3mf"))]
fn save_3mf(_assembly: &Assembly, path: &Path) -> AssemblyResult<()> {
    Err(AssemblyError::Io {
        path: path.to_path_buf(),
        source: std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "3MF export requires the 'export-3mf' feature",
        ),
    })
}

/// Generate 3MF model XML for the assembly.
#[cfg(feature = "export-3mf")]
#[allow(clippy::format_push_string)]
fn generate_3mf_model_xml(assembly: &Assembly) -> String {
    use nalgebra::Isometry3;

    let part_count = assembly.part_count();
    let mut xml = String::with_capacity(part_count * 1000);

    xml.push_str(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <metadata name="Title">"#,
    );
    xml.push_str(&escape_xml(assembly.name()));
    xml.push_str("</metadata>\n");

    if let Some(version) = assembly.version() {
        xml.push_str("  <metadata name=\"Version\">");
        xml.push_str(&escape_xml(version));
        xml.push_str("</metadata>\n");
    }

    xml.push_str("  <resources>\n");

    // Create a stable ordering for parts
    let mut part_ids: Vec<&str> = assembly.part_ids().collect();
    part_ids.sort_unstable();

    // Write each part as a separate object
    for (obj_id, part_id) in part_ids.iter().enumerate() {
        if let Some(part) = assembly.get_part(part_id) {
            let object_id = obj_id + 1; // 3MF IDs start at 1

            xml.push_str(&format!(
                "    <object id=\"{}\" type=\"model\" name=\"{}\">\n",
                object_id,
                escape_xml(part.id())
            ));
            xml.push_str("      <mesh>\n        <vertices>\n");

            // Write vertices
            for v in &part.mesh().vertices {
                xml.push_str(&format!(
                    "          <vertex x=\"{:.6}\" y=\"{:.6}\" z=\"{:.6}\"/>\n",
                    v.position.x, v.position.y, v.position.z
                ));
            }

            xml.push_str("        </vertices>\n        <triangles>\n");

            // Write triangles
            for face in &part.mesh().faces {
                xml.push_str(&format!(
                    "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
                    face[0], face[1], face[2]
                ));
            }

            xml.push_str("        </triangles>\n      </mesh>\n    </object>\n");
        }
    }

    xml.push_str("  </resources>\n  <build>\n");

    // Write build items with transforms
    for (obj_id, part_id) in part_ids.iter().enumerate() {
        let object_id = obj_id + 1;

        // Get world transform for this part
        let world_transform = assembly
            .get_world_transform(part_id)
            .unwrap_or_else(Isometry3::identity);

        // Only include transform attribute if it's not identity
        if is_identity_transform(&world_transform) {
            xml.push_str(&format!("    <item objectid=\"{object_id}\"/>\n"));
        } else {
            // 3MF uses a 3x4 affine matrix (row-major)
            let matrix = transform_to_3mf_matrix(&world_transform);
            xml.push_str(&format!(
                "    <item objectid=\"{object_id}\" transform=\"{matrix}\"/>\n"
            ));
        }
    }

    xml.push_str("  </build>\n</model>\n");

    xml
}

/// Content types XML for 3MF.
#[cfg(feature = "export-3mf")]
const CONTENT_TYPES_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"#;

/// Relationships XML for 3MF.
#[cfg(feature = "export-3mf")]
const RELS_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#;

/// Check if a transform is approximately identity.
#[cfg(feature = "export-3mf")]
fn is_identity_transform(t: &nalgebra::Isometry3<f64>) -> bool {
    let eps = 1e-10;
    let translation_zero = t.translation.vector.norm() < eps;
    let rotation_identity =
        (t.rotation.angle() < eps) || (t.rotation.angle() - std::f64::consts::TAU).abs() < eps;
    translation_zero && rotation_identity
}

/// Convert an Isometry3 to a 3MF transform matrix string.
#[cfg(feature = "export-3mf")]
fn transform_to_3mf_matrix(t: &nalgebra::Isometry3<f64>) -> String {
    // 3MF uses a 3x4 affine matrix in row-major order:
    // m00 m01 m02 m03 m10 m11 m12 m13 m20 m21 m22 m23
    let rot = t.rotation.to_rotation_matrix();
    let trans = t.translation.vector;

    format!(
        "{:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
        rot[(0, 0)],
        rot[(0, 1)],
        rot[(0, 2)],
        trans.x,
        rot[(1, 0)],
        rot[(1, 1)],
        rot[(1, 2)],
        trans.y,
        rot[(2, 0)],
        rot[(2, 1)],
        rot[(2, 2)],
        trans.z
    )
}

/// Escape special XML characters.
#[cfg(feature = "export-3mf")]
fn escape_xml(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

/// Sanitize a string for use as a filename.
fn sanitize_filename(s: &str) -> String {
    s.chars()
        .map(|c| match c {
            '/' | '\\' | ':' | '*' | '?' | '"' | '<' | '>' | '|' => '_',
            _ => c,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_from_path() {
        assert_eq!(
            AssemblyExportFormat::from_path(Path::new("test.3mf")),
            Some(AssemblyExportFormat::ThreeMf)
        );
        assert_eq!(
            AssemblyExportFormat::from_path(Path::new("test.stl")),
            Some(AssemblyExportFormat::StlMerged)
        );
        assert_eq!(
            AssemblyExportFormat::from_path(Path::new("test.STL")),
            Some(AssemblyExportFormat::StlMerged)
        );
        assert_eq!(
            AssemblyExportFormat::from_path(Path::new("test.obj")),
            None
        );
    }

    #[test]
    fn test_format_extension() {
        assert_eq!(AssemblyExportFormat::ThreeMf.extension(), "3mf");
        assert_eq!(AssemblyExportFormat::StlMerged.extension(), "stl");
        assert_eq!(AssemblyExportFormat::StlSeparate.extension(), "stl");
    }

    #[test]
    fn test_sanitize_filename() {
        assert_eq!(sanitize_filename("normal_name"), "normal_name");
        assert_eq!(sanitize_filename("with/slash"), "with_slash");
        assert_eq!(sanitize_filename("with:colon"), "with_colon");
        assert_eq!(
            sanitize_filename("with*star?question"),
            "with_star_question"
        );
    }

    #[cfg(feature = "export-3mf")]
    #[test]
    fn test_escape_xml() {
        assert_eq!(escape_xml("a < b"), "a &lt; b");
        assert_eq!(escape_xml("a & b"), "a &amp; b");
        assert_eq!(escape_xml("\"test\""), "&quot;test&quot;");
        assert_eq!(escape_xml("'test'"), "&apos;test&apos;");
    }

    #[cfg(feature = "export-3mf")]
    #[test]
    fn test_identity_transform() {
        use nalgebra::Isometry3;

        let identity = Isometry3::identity();
        assert!(is_identity_transform(&identity));

        let translated = Isometry3::translation(1.0, 0.0, 0.0);
        assert!(!is_identity_transform(&translated));
    }

    #[cfg(feature = "export-3mf")]
    #[test]
    fn test_transform_to_matrix() {
        use nalgebra::Isometry3;

        let translated = Isometry3::translation(1.0, 2.0, 3.0);
        let matrix_str = transform_to_3mf_matrix(&translated);

        // Should have 12 numbers: 3x4 affine matrix
        let parts: Vec<&str> = matrix_str.split_whitespace().collect();
        assert_eq!(parts.len(), 12);
    }
}
