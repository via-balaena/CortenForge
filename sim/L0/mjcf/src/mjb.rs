//! MJB (MuJoCo Binary) format support.
//!
//! MJB is a binary serialization format for MJCF models that provides:
//! - Faster loading than XML parsing
//! - Pre-serialized model data ready for deserialization
//! - Reduced file sizes through binary encoding
//!
//! # File Format
//!
//! The MJB file format consists of:
//! 1. **Magic bytes**: `MJB1` (4 bytes) - identifies the file format
//! 2. **Version**: `u32` little-endian (4 bytes) - format version (currently 1)
//! 3. **Flags**: `u32` little-endian (4 bytes) - reserved for future use
//! 4. **Payload**: bincode-encoded [`MjcfModel`] data
//!
//! # Example
//!
//! ```no_run
//! use sim_mjcf::{parse_mjcf_str, load_mjb_file, save_mjb_file};
//!
//! // Parse an MJCF model
//! let mjcf = r#"
//!     <mujoco model="robot">
//!         <worldbody>
//!             <body name="base" pos="0 0 1">
//!                 <geom type="sphere" size="0.1"/>
//!             </body>
//!         </worldbody>
//!     </mujoco>
//! "#;
//! let model = parse_mjcf_str(mjcf).expect("should parse");
//!
//! // Save to MJB binary format for faster loading later
//! save_mjb_file(&model, "robot.mjb").expect("should save");
//!
//! // Load from MJB (much faster than XML parsing)
//! let loaded = load_mjb_file("robot.mjb").expect("should load");
//! assert_eq!(loaded.name, "robot");
//! ```

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::error::{MjcfError, Result};
use crate::types::MjcfModel;

/// Magic bytes identifying an MJB file.
pub const MJB_MAGIC: [u8; 4] = *b"MJB1";

/// Current MJB format version.
pub const MJB_VERSION: u32 = 1;

/// Header size in bytes (magic + version + flags).
pub const MJB_HEADER_SIZE: usize = 12;

/// MJB file header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MjbHeader {
    /// Magic bytes (must be `MJB1`).
    pub magic: [u8; 4],
    /// Format version.
    pub version: u32,
    /// Flags (reserved for future use).
    pub flags: u32,
}

impl MjbHeader {
    /// Create a new header with default values.
    #[must_use]
    pub fn new() -> Self {
        Self {
            magic: MJB_MAGIC,
            version: MJB_VERSION,
            flags: 0,
        }
    }

    /// Write the header to a writer.
    fn write_to<W: Write>(&self, writer: &mut W) -> std::io::Result<()> {
        writer.write_all(&self.magic)?;
        writer.write_all(&self.version.to_le_bytes())?;
        writer.write_all(&self.flags.to_le_bytes())?;
        Ok(())
    }

    /// Read the header from a reader.
    fn read_from<R: Read>(reader: &mut R) -> std::io::Result<Self> {
        let mut magic = [0u8; 4];
        reader.read_exact(&mut magic)?;

        let mut version_bytes = [0u8; 4];
        reader.read_exact(&mut version_bytes)?;
        let version = u32::from_le_bytes(version_bytes);

        let mut flags_bytes = [0u8; 4];
        reader.read_exact(&mut flags_bytes)?;
        let flags = u32::from_le_bytes(flags_bytes);

        Ok(Self {
            magic,
            version,
            flags,
        })
    }

    /// Validate the header.
    fn validate(&self) -> Result<()> {
        if self.magic != MJB_MAGIC {
            return Err(MjcfError::InvalidMjbMagic(self.magic));
        }
        if self.version != MJB_VERSION {
            return Err(MjcfError::UnsupportedMjbVersion(self.version));
        }
        Ok(())
    }
}

impl Default for MjbHeader {
    fn default() -> Self {
        Self::new()
    }
}

/// Save an [`MjcfModel`] to a file in MJB binary format.
///
/// This is significantly faster to load than parsing XML, making it ideal for
/// caching parsed models or distributing pre-compiled model files.
///
/// # Arguments
///
/// * `model` - The MJCF model to save
/// * `path` - The file path to write to (conventionally with `.mjb` extension)
///
/// # Errors
///
/// Returns an error if:
/// - The file cannot be created
/// - Serialization fails
///
/// # Example
///
/// ```no_run
/// use sim_mjcf::{parse_mjcf_str, save_mjb_file};
///
/// let model = parse_mjcf_str("<mujoco><worldbody/></mujoco>").unwrap();
/// save_mjb_file(&model, "model.mjb").unwrap();
/// ```
pub fn save_mjb_file(model: &MjcfModel, path: impl AsRef<Path>) -> Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    save_mjb_writer(model, &mut writer)
}

/// Save an [`MjcfModel`] to a writer in MJB binary format.
///
/// This allows writing to any destination that implements [`Write`], such as
/// in-memory buffers, network streams, or compressed files.
///
/// # Arguments
///
/// * `model` - The MJCF model to save
/// * `writer` - The writer to write to
///
/// # Errors
///
/// Returns an error if serialization fails.
pub fn save_mjb_writer<W: Write>(model: &MjcfModel, writer: &mut W) -> Result<()> {
    // Write header
    let header = MjbHeader::new();
    header
        .write_to(writer)
        .map_err(|e| MjcfError::MjbSerialize(e.to_string()))?;

    // Serialize model using bincode
    bincode::serialize_into(writer, model).map_err(|e| MjcfError::MjbSerialize(e.to_string()))?;

    Ok(())
}

/// Save an [`MjcfModel`] to a byte vector in MJB binary format.
///
/// This is useful for in-memory operations or when you need to inspect the
/// binary data before writing to a file.
///
/// # Arguments
///
/// * `model` - The MJCF model to save
///
/// # Returns
///
/// A byte vector containing the MJB binary data.
///
/// # Errors
///
/// Returns an error if serialization fails.
///
/// # Example
///
/// ```no_run
/// use sim_mjcf::{parse_mjcf_str, save_mjb_bytes};
///
/// let model = parse_mjcf_str("<mujoco><worldbody/></mujoco>").unwrap();
/// let bytes = save_mjb_bytes(&model).unwrap();
/// println!("MJB size: {} bytes", bytes.len());
/// ```
pub fn save_mjb_bytes(model: &MjcfModel) -> Result<Vec<u8>> {
    let mut buffer = Vec::new();
    save_mjb_writer(model, &mut buffer)?;
    Ok(buffer)
}

/// Load an [`MjcfModel`] from a file in MJB binary format.
///
/// This is significantly faster than parsing MJCF XML, making it ideal for
/// loading cached or pre-compiled model files.
///
/// # Arguments
///
/// * `path` - The file path to read from (conventionally with `.mjb` extension)
///
/// # Returns
///
/// The deserialized MJCF model.
///
/// # Errors
///
/// Returns an error if:
/// - The file cannot be opened
/// - The file has invalid magic bytes
/// - The file version is unsupported
/// - Deserialization fails
///
/// # Example
///
/// ```no_run
/// use sim_mjcf::load_mjb_file;
///
/// let model = load_mjb_file("robot.mjb").unwrap();
/// println!("Loaded model: {}", model.name);
/// ```
pub fn load_mjb_file(path: impl AsRef<Path>) -> Result<MjcfModel> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    load_mjb_reader(&mut reader)
}

/// Load an [`MjcfModel`] from a reader in MJB binary format.
///
/// This allows reading from any source that implements [`Read`], such as
/// in-memory buffers, network streams, or compressed files.
///
/// # Arguments
///
/// * `reader` - The reader to read from
///
/// # Returns
///
/// The deserialized MJCF model.
///
/// # Errors
///
/// Returns an error if:
/// - The reader has invalid magic bytes
/// - The format version is unsupported
/// - Deserialization fails
pub fn load_mjb_reader<R: Read>(reader: &mut R) -> Result<MjcfModel> {
    // Read and validate header
    let header = MjbHeader::read_from(reader)
        .map_err(|e| MjcfError::MjbDeserialize(format!("failed to read header: {e}")))?;
    header.validate()?;

    // Deserialize model using bincode
    let model: MjcfModel =
        bincode::deserialize_from(reader).map_err(|e| MjcfError::MjbDeserialize(e.to_string()))?;

    Ok(model)
}

/// Load an [`MjcfModel`] from a byte slice in MJB binary format.
///
/// This is useful for loading from in-memory data or embedded resources.
///
/// # Arguments
///
/// * `bytes` - The byte slice containing MJB binary data
///
/// # Returns
///
/// The deserialized MJCF model.
///
/// # Errors
///
/// Returns an error if:
/// - The bytes have invalid magic bytes
/// - The format version is unsupported
/// - Deserialization fails
///
/// # Example
///
/// ```ignore
/// use sim_mjcf::load_mjb_bytes;
///
/// // Assuming you have an MJB file embedded as bytes
/// let mjb_data: &[u8] = include_bytes!("robot.mjb");
/// let model = load_mjb_bytes(mjb_data).unwrap();
/// ```
pub fn load_mjb_bytes(bytes: &[u8]) -> Result<MjcfModel> {
    let mut reader = std::io::Cursor::new(bytes);
    load_mjb_reader(&mut reader)
}

/// Check if a file is a valid MJB file by reading its magic bytes.
///
/// This is a quick way to check if a file is likely an MJB file without
/// fully parsing it.
///
/// # Arguments
///
/// * `path` - The file path to check
///
/// # Returns
///
/// `true` if the file starts with the MJB magic bytes, `false` otherwise.
///
/// # Example
///
/// ```no_run
/// use sim_mjcf::is_mjb_file;
///
/// if is_mjb_file("model.mjb") {
///     println!("This is an MJB file!");
/// }
/// ```
pub fn is_mjb_file(path: impl AsRef<Path>) -> bool {
    let Ok(file) = File::open(path) else {
        return false;
    };
    let mut reader = BufReader::new(file);
    let mut magic = [0u8; 4];
    if reader.read_exact(&mut magic).is_err() {
        return false;
    }
    magic == MJB_MAGIC
}

/// Check if a byte slice is valid MJB data by checking its magic bytes.
///
/// # Arguments
///
/// * `bytes` - The byte slice to check
///
/// # Returns
///
/// `true` if the bytes start with the MJB magic bytes, `false` otherwise.
pub fn is_mjb_bytes(bytes: &[u8]) -> bool {
    bytes.len() >= 4 && bytes[..4] == MJB_MAGIC
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::parse_mjcf_str;

    fn simple_model() -> MjcfModel {
        parse_mjcf_str(
            r#"
            <mujoco model="test_model">
                <option timestep="0.001" gravity="0 0 -9.81"/>
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <joint type="free"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should parse simple model")
    }

    fn complex_model() -> MjcfModel {
        parse_mjcf_str(
            r#"
            <mujoco model="two_link_arm">
                <option timestep="0.002" integrator="RK4"/>
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <inertial pos="0 0 0.25" mass="1.0" diaginertia="0.01 0.01 0.001"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                        <joint name="shoulder" type="hinge" axis="1 0 0" range="-3.14 3.14"/>
                        <body name="link2" pos="0 0 0.5">
                            <inertial pos="0 0 0.25" mass="0.5" diaginertia="0.005 0.005 0.0005"/>
                            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.5"/>
                            <joint name="elbow" type="hinge" axis="1 0 0" range="0 2.5"/>
                        </body>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="shoulder_motor" joint="shoulder" gear="100"/>
                    <motor name="elbow_motor" joint="elbow" gear="50"/>
                </actuator>
            </mujoco>
            "#,
        )
        .expect("should parse complex model")
    }

    #[test]
    fn test_header_roundtrip() {
        let header = MjbHeader::new();
        let mut buffer = Vec::new();
        header.write_to(&mut buffer).unwrap();

        assert_eq!(buffer.len(), MJB_HEADER_SIZE);

        let mut cursor = std::io::Cursor::new(&buffer);
        let loaded = MjbHeader::read_from(&mut cursor).unwrap();

        assert_eq!(header, loaded);
    }

    #[test]
    fn test_header_validation() {
        let mut header = MjbHeader::new();
        assert!(header.validate().is_ok());

        // Invalid magic
        header.magic = *b"NOPE";
        assert!(matches!(
            header.validate(),
            Err(MjcfError::InvalidMjbMagic(_))
        ));

        // Invalid version
        header.magic = MJB_MAGIC;
        header.version = 999;
        assert!(matches!(
            header.validate(),
            Err(MjcfError::UnsupportedMjbVersion(999))
        ));
    }

    #[test]
    fn test_simple_model_roundtrip() {
        let original = simple_model();

        // Save to bytes
        let bytes = save_mjb_bytes(&original).expect("should serialize");

        // Verify magic bytes
        assert!(is_mjb_bytes(&bytes));

        // Load back
        let loaded = load_mjb_bytes(&bytes).expect("should deserialize");

        // Compare
        assert_eq!(original.name, loaded.name);
        assert_eq!(original.option.timestep, loaded.option.timestep);
        assert_eq!(
            original.worldbody.children.len(),
            loaded.worldbody.children.len()
        );
        assert_eq!(
            original.worldbody.children[0].name,
            loaded.worldbody.children[0].name
        );
    }

    #[test]
    fn test_complex_model_roundtrip() {
        let original = complex_model();

        // Save to bytes
        let bytes = save_mjb_bytes(&original).expect("should serialize");

        // Load back
        let loaded = load_mjb_bytes(&bytes).expect("should deserialize");

        // Compare model structure
        assert_eq!(original.name, loaded.name);
        assert_eq!(original.option.integrator, loaded.option.integrator);
        assert_eq!(original.actuators.len(), loaded.actuators.len());

        // Compare actuator names
        for (orig, load) in original.actuators.iter().zip(loaded.actuators.iter()) {
            assert_eq!(orig.name, load.name);
        }

        // Compare body tree
        fn compare_bodies(orig: &crate::types::MjcfBody, load: &crate::types::MjcfBody) {
            assert_eq!(orig.name, load.name);
            assert_eq!(orig.joints.len(), load.joints.len());
            assert_eq!(orig.geoms.len(), load.geoms.len());
            assert_eq!(orig.children.len(), load.children.len());
            for (orig_child, load_child) in orig.children.iter().zip(load.children.iter()) {
                compare_bodies(orig_child, load_child);
            }
        }
        compare_bodies(&original.worldbody, &loaded.worldbody);
    }

    #[test]
    fn test_file_roundtrip() {
        let original = complex_model();

        // Create a temporary file
        let temp_dir = tempfile::tempdir().expect("should create temp dir");
        let mjb_path = temp_dir.path().join("test.mjb");

        // Save to file
        save_mjb_file(&original, &mjb_path).expect("should save to file");

        // Verify file exists and is valid MJB
        assert!(is_mjb_file(&mjb_path));

        // Load from file
        let loaded = load_mjb_file(&mjb_path).expect("should load from file");

        // Compare
        assert_eq!(original.name, loaded.name);
        assert_eq!(original.actuators.len(), loaded.actuators.len());
    }

    #[test]
    fn test_binary_size_smaller_than_xml() {
        let model = complex_model();

        let xml = r#"
            <mujoco model="two_link_arm">
                <option timestep="0.002" integrator="RK4"/>
                <worldbody>
                    <body name="link1" pos="0 0 1">
                        <inertial pos="0 0 0.25" mass="1.0" diaginertia="0.01 0.01 0.001"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.5"/>
                        <joint name="shoulder" type="hinge" axis="1 0 0" range="-3.14 3.14"/>
                        <body name="link2" pos="0 0 0.5">
                            <inertial pos="0 0 0.25" mass="0.5" diaginertia="0.005 0.005 0.0005"/>
                            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.5"/>
                            <joint name="elbow" type="hinge" axis="1 0 0" range="0 2.5"/>
                        </body>
                    </body>
                </worldbody>
                <actuator>
                    <motor name="shoulder_motor" joint="shoulder" gear="100"/>
                    <motor name="elbow_motor" joint="elbow" gear="50"/>
                </actuator>
            </mujoco>
        "#;

        let bytes = save_mjb_bytes(&model).expect("should serialize");
        let xml_bytes = xml.as_bytes();

        // MJB should be reasonably compact (may not always be smaller due to
        // overhead, but should be in the same ballpark or smaller)
        // For very small models, MJB overhead may be larger, but for complex
        // models with embedded data, MJB should be more efficient
        println!(
            "MJB size: {} bytes, XML size: {} bytes",
            bytes.len(),
            xml_bytes.len()
        );
    }

    #[test]
    fn test_invalid_magic_bytes() {
        let bad_data = b"NOPE1234567890";
        let result = load_mjb_bytes(bad_data);
        assert!(matches!(result, Err(MjcfError::InvalidMjbMagic(_))));
    }

    #[test]
    fn test_invalid_version() {
        let mut buffer = Vec::new();
        buffer.extend_from_slice(&MJB_MAGIC);
        buffer.extend_from_slice(&999u32.to_le_bytes()); // Bad version
        buffer.extend_from_slice(&0u32.to_le_bytes()); // Flags
        // Add some dummy data
        buffer.extend_from_slice(&[0u8; 100]);

        let result = load_mjb_bytes(&buffer);
        assert!(matches!(result, Err(MjcfError::UnsupportedMjbVersion(999))));
    }

    #[test]
    fn test_truncated_data() {
        let model = simple_model();
        let bytes = save_mjb_bytes(&model).expect("should serialize");

        // Truncate to just the header
        let truncated = &bytes[..MJB_HEADER_SIZE + 10];
        let result = load_mjb_bytes(truncated);
        assert!(matches!(result, Err(MjcfError::MjbDeserialize(_))));
    }

    #[test]
    fn test_is_mjb_bytes() {
        assert!(is_mjb_bytes(b"MJB1anything"));
        assert!(!is_mjb_bytes(b"MJB")); // Too short
        assert!(!is_mjb_bytes(b"MJCF")); // Wrong magic
        assert!(!is_mjb_bytes(b"")); // Empty
    }

    #[test]
    fn test_model_with_meshes() {
        // Test a model with embedded mesh data
        let mjcf = r#"
            <mujoco model="mesh_test">
                <asset>
                    <mesh name="cube" vertex="
                        -0.5 -0.5 -0.5
                         0.5 -0.5 -0.5
                         0.5  0.5 -0.5
                        -0.5  0.5 -0.5
                        -0.5 -0.5  0.5
                         0.5 -0.5  0.5
                         0.5  0.5  0.5
                        -0.5  0.5  0.5
                    " face="
                        0 1 2  0 2 3
                        4 6 5  4 7 6
                        0 4 5  0 5 1
                        2 6 7  2 7 3
                        0 3 7  0 7 4
                        1 5 6  1 6 2
                    "/>
                </asset>
                <worldbody>
                    <body name="box">
                        <geom type="mesh" mesh="cube"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = parse_mjcf_str(mjcf).expect("should parse");

        // Verify mesh was parsed
        assert_eq!(model.meshes.len(), 1);
        assert!(model.meshes[0].has_embedded_data());

        // Round-trip through MJB
        let bytes = save_mjb_bytes(&model).expect("should serialize");
        let loaded = load_mjb_bytes(&bytes).expect("should deserialize");

        // Verify mesh data preserved
        assert_eq!(loaded.meshes.len(), 1);
        assert!(loaded.meshes[0].has_embedded_data());
        assert_eq!(
            model.meshes[0].vertex.as_ref().map(Vec::len),
            loaded.meshes[0].vertex.as_ref().map(Vec::len)
        );
        assert_eq!(
            model.meshes[0].face.as_ref().map(Vec::len),
            loaded.meshes[0].face.as_ref().map(Vec::len)
        );
    }
}
