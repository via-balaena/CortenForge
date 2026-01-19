//! Point cloud file I/O operations.
//!
//! Supports reading and writing point clouds in XYZ format:
//! - **XYZ** - Simple ASCII format (x y z per line, optionally with normals and colors)
//!
//! # Example
//!
//! ```no_run
//! use mesh_scan::pointcloud::PointCloud;
//!
//! // Load a point cloud
//! let cloud = PointCloud::load("scan.xyz").unwrap();
//!
//! // Save to a different file
//! cloud.save("output.xyz").unwrap();
//! ```

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

use nalgebra::{Point3, Vector3};

use super::{CloudPoint, PointCloud};
use crate::error::{ScanError, ScanResult};
use mesh_types::VertexColor;

/// Supported point cloud file formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointCloudFormat {
    /// Simple XYZ ASCII format.
    Xyz,
}

impl PointCloudFormat {
    /// Detects the format from a file extension.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::io::PointCloudFormat;
    ///
    /// assert_eq!(
    ///     PointCloudFormat::from_extension("xyz"),
    ///     Some(PointCloudFormat::Xyz)
    /// );
    /// assert_eq!(PointCloudFormat::from_extension("unknown"), None);
    /// ```
    #[must_use]
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_lowercase().as_str() {
            "xyz" | "txt" | "pts" => Some(Self::Xyz),
            _ => None,
        }
    }

    /// Detects the format from a file path.
    #[must_use]
    pub fn from_path<P: AsRef<Path>>(path: P) -> Option<Self> {
        path.as_ref()
            .extension()
            .and_then(|ext| ext.to_str())
            .and_then(Self::from_extension)
    }
}

impl PointCloud {
    /// Loads a point cloud from a file.
    ///
    /// The format is detected from the file extension.
    /// Currently supports XYZ/TXT/PTS formats.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be read or the format is unsupported.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_scan::pointcloud::PointCloud;
    ///
    /// let cloud = PointCloud::load("scan.xyz").unwrap();
    /// ```
    pub fn load<P: AsRef<Path>>(path: P) -> ScanResult<Self> {
        let path = path.as_ref();
        let _format =
            PointCloudFormat::from_path(path).ok_or_else(|| ScanError::UnsupportedFormat {
                format: path
                    .extension()
                    .and_then(|e| e.to_str())
                    .unwrap_or("unknown")
                    .to_string(),
            })?;

        load_xyz(path)
    }

    /// Saves the point cloud to a file.
    ///
    /// The format is detected from the file extension.
    /// Currently supports XYZ/TXT/PTS formats.
    ///
    /// # Errors
    ///
    /// Returns an error if the file cannot be written or the format is unsupported.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// let cloud = PointCloud::from_positions(&[Point3::new(0.0, 0.0, 0.0)]);
    /// cloud.save("output.xyz").unwrap();
    /// ```
    pub fn save<P: AsRef<Path>>(&self, path: P) -> ScanResult<()> {
        let path = path.as_ref();
        let _format =
            PointCloudFormat::from_path(path).ok_or_else(|| ScanError::UnsupportedFormat {
                format: path
                    .extension()
                    .and_then(|e| e.to_str())
                    .unwrap_or("unknown")
                    .to_string(),
            })?;

        save_xyz(self, path)
    }
}

/// Loads a point cloud from an XYZ file.
fn load_xyz<P: AsRef<Path>>(path: P) -> ScanResult<PointCloud> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);

    let mut points = Vec::new();

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();

        // Skip empty lines and comments
        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();

        if parts.len() < 3 {
            continue; // Skip malformed lines
        }

        let x = parts[0]
            .parse::<f64>()
            .map_err(|_| ScanError::InvalidParameter {
                reason: format!("invalid x coordinate: {}", parts[0]),
            })?;
        let y = parts[1]
            .parse::<f64>()
            .map_err(|_| ScanError::InvalidParameter {
                reason: format!("invalid y coordinate: {}", parts[1]),
            })?;
        let z = parts[2]
            .parse::<f64>()
            .map_err(|_| ScanError::InvalidParameter {
                reason: format!("invalid z coordinate: {}", parts[2]),
            })?;

        let mut point = CloudPoint::new(Point3::new(x, y, z));

        // Try to read normals (nx, ny, nz at positions 3, 4, 5)
        if parts.len() >= 6 {
            if let (Ok(nx), Ok(ny), Ok(nz)) = (
                parts[3].parse::<f64>(),
                parts[4].parse::<f64>(),
                parts[5].parse::<f64>(),
            ) {
                point.normal = Some(Vector3::new(nx, ny, nz));
            }
        }

        // Try to read colors (r, g, b at positions 6, 7, 8 or 3, 4, 5 if no normals)
        let color_start = if point.normal.is_some() { 6 } else { 3 };
        if parts.len() >= color_start + 3 {
            if let (Ok(r), Ok(g), Ok(b)) = (
                parts[color_start].parse::<u8>(),
                parts[color_start + 1].parse::<u8>(),
                parts[color_start + 2].parse::<u8>(),
            ) {
                point.color = Some(VertexColor::new(r, g, b));
            }
        }

        points.push(point);
    }

    Ok(PointCloud { points })
}

/// Saves a point cloud to an XYZ file.
fn save_xyz<P: AsRef<Path>>(cloud: &PointCloud, path: P) -> ScanResult<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writeln!(writer, "# Point cloud generated by mesh-scan")?;
    writeln!(writer, "# Format: x y z [nx ny nz] [r g b]")?;

    let has_normals = cloud.has_normals();
    let has_colors = cloud.has_colors();

    for point in &cloud.points {
        write!(
            writer,
            "{} {} {}",
            point.position.x, point.position.y, point.position.z
        )?;

        if has_normals {
            if let Some(normal) = point.normal {
                write!(writer, " {} {} {}", normal.x, normal.y, normal.z)?;
            } else {
                write!(writer, " 0 0 0")?;
            }
        }

        if has_colors {
            if let Some(color) = point.color {
                write!(writer, " {} {} {}", color.r, color.g, color.b)?;
            } else {
                write!(writer, " 0 0 0")?;
            }
        }

        writeln!(writer)?;
    }

    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::redundant_clone,
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use tempfile::tempdir;

    fn make_test_cloud() -> PointCloud {
        let mut cloud = PointCloud::new();
        cloud.push(CloudPoint::from_coords(0.0, 0.0, 0.0));
        cloud.push(CloudPoint::from_coords(1.0, 0.0, 0.0));
        cloud.push(CloudPoint::from_coords(0.0, 1.0, 0.0));
        cloud
    }

    fn make_test_cloud_with_normals() -> PointCloud {
        let mut cloud = PointCloud::new();
        cloud.push(CloudPoint::with_normal(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ));
        cloud.push(CloudPoint::with_normal(
            Point3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ));
        cloud
    }

    fn make_test_cloud_with_colors() -> PointCloud {
        let mut cloud = PointCloud::new();
        cloud.push(CloudPoint::with_normal_and_color(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            VertexColor::new(255, 0, 0),
        ));
        cloud.push(CloudPoint::with_normal_and_color(
            Point3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            VertexColor::new(0, 255, 0),
        ));
        cloud
    }

    #[test]
    fn test_format_from_extension() {
        assert_eq!(
            PointCloudFormat::from_extension("xyz"),
            Some(PointCloudFormat::Xyz)
        );
        assert_eq!(
            PointCloudFormat::from_extension("XYZ"),
            Some(PointCloudFormat::Xyz)
        );
        assert_eq!(
            PointCloudFormat::from_extension("txt"),
            Some(PointCloudFormat::Xyz)
        );
        assert_eq!(
            PointCloudFormat::from_extension("pts"),
            Some(PointCloudFormat::Xyz)
        );
        assert_eq!(PointCloudFormat::from_extension("obj"), None);
    }

    #[test]
    fn test_format_from_path() {
        assert_eq!(
            PointCloudFormat::from_path("test.xyz"),
            Some(PointCloudFormat::Xyz)
        );
        assert_eq!(
            PointCloudFormat::from_path("/path/to/file.txt"),
            Some(PointCloudFormat::Xyz)
        );
        assert_eq!(PointCloudFormat::from_path("noextension"), None);
    }

    #[test]
    fn test_xyz_roundtrip() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("test.xyz");

        let cloud = make_test_cloud();
        cloud.save(&path).unwrap();

        let loaded = PointCloud::load(&path).unwrap();
        assert_eq!(loaded.len(), cloud.len());

        for (original, loaded) in cloud.points.iter().zip(loaded.points.iter()) {
            assert_relative_eq!(original.position.x, loaded.position.x, epsilon = 1e-10);
            assert_relative_eq!(original.position.y, loaded.position.y, epsilon = 1e-10);
            assert_relative_eq!(original.position.z, loaded.position.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_xyz_roundtrip_with_normals() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("test_normals.xyz");

        let cloud = make_test_cloud_with_normals();
        cloud.save(&path).unwrap();

        let loaded = PointCloud::load(&path).unwrap();
        assert_eq!(loaded.len(), cloud.len());

        for (original, loaded) in cloud.points.iter().zip(loaded.points.iter()) {
            assert!(loaded.normal.is_some());
            let orig_n = original.normal.unwrap();
            let load_n = loaded.normal.unwrap();
            assert_relative_eq!(orig_n.x, load_n.x, epsilon = 1e-10);
            assert_relative_eq!(orig_n.y, load_n.y, epsilon = 1e-10);
            assert_relative_eq!(orig_n.z, load_n.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_xyz_roundtrip_with_colors() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("test_colors.xyz");

        let cloud = make_test_cloud_with_colors();
        cloud.save(&path).unwrap();

        let loaded = PointCloud::load(&path).unwrap();
        assert_eq!(loaded.len(), cloud.len());

        for (original, loaded) in cloud.points.iter().zip(loaded.points.iter()) {
            assert!(loaded.color.is_some());
            let orig_c = original.color.unwrap();
            let load_c = loaded.color.unwrap();
            assert_eq!(orig_c.r, load_c.r);
            assert_eq!(orig_c.g, load_c.g);
            assert_eq!(orig_c.b, load_c.b);
        }
    }

    #[test]
    fn test_unsupported_format() {
        let result = PointCloud::load("test.unknown");
        assert!(matches!(result, Err(ScanError::UnsupportedFormat { .. })));
    }

    #[test]
    fn test_file_not_found() {
        let result = PointCloud::load("nonexistent.xyz");
        assert!(matches!(result, Err(ScanError::IoError(_))));
    }

    #[test]
    fn test_xyz_with_comments() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("comments.xyz");

        // Write a file with comments
        let mut file = File::create(&path).unwrap();
        writeln!(file, "# This is a comment").unwrap();
        writeln!(file, "// Another comment").unwrap();
        writeln!(file).unwrap();
        writeln!(file, "0 0 0").unwrap();
        writeln!(file, "1 1 1").unwrap();
        drop(file);

        let cloud = PointCloud::load(&path).unwrap();
        assert_eq!(cloud.len(), 2);
    }

    #[test]
    fn test_empty_cloud_save() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("empty.xyz");

        let cloud = PointCloud::new();
        cloud.save(&path).unwrap();

        let loaded = PointCloud::load(&path).unwrap();
        assert!(loaded.is_empty());
    }
}
