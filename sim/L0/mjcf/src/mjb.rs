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
//! 2. **Version**: `u32` little-endian (4 bytes) - format version ([`MJB_VERSION`])
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
//!
//! # Limits
//!
//! The readers take a file whose strings and byte buffers claim at most
//! [`MJB_DECODE_LIMIT`] bytes together, each length claimed before its buffer
//! is allocated ([`MjcfError::MjbTooLarge`]), and whose body and frame tree is
//! no deeper than the loader takes, a level past it refused before it is read
//! ([`MjcfError::TooDeep`]). The writers refuse such a tree before encoding
//! it. Both decode and encode on the loader's own stack (`stack`), as the XML
//! loaders do.

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::error::{MjcfError, Result};
use crate::stack::{MAX_BODY_TREE_DEPTH, decoding, on_large_stack};
use crate::types::MjcfModel;
use crate::validation::check_tree_depth;

/// Magic bytes identifying an MJB file.
pub const MJB_MAGIC: [u8; 4] = *b"MJB1";

/// Current MJB format version.
///
/// 14 since Rigid-loading PR RL-7b: `MjcfBody` and `MjcfFrame` took `items`,
/// and `MjcfFrame` `composites`.
/// 13 since Rigid-loading PR RL-6a: `MjcfTendon` took `armature`.
/// 12 since Rigid-loading PR RL-5c: `MjcfGeom`'s `size` optional, a prefix of
/// up to three, and `MjcfGeomDefaults` took `size`.
/// 11 since Rigid-loading PR RL-5b: `MjcfActuator`'s `kp`, `area`, `bias`,
/// muscle values and `gain`, and `MjcfSensor`'s `noise` and `cutoff`, optional;
/// `MjcfDefault`'s actuator elements a list of `MjcfActuator`, which took
/// `tausmooth`.
/// 10 since Rigid-loading PR RL-5a: `MjcfMesh` took `class`, `MjcfComposite`
/// `site`, `MjcfJoint` `freejoint`.
/// 9 since Rigid-loading PR RL-4b: `MjcfBody` took `xyaxes` and `zaxis`.
/// 8 since Rigid-loading PR RL-4a: `MjcfActuator` and `MjcfActuatorDefaults`
/// took `inheritrange`, `MjcfActuatorType` `IntVelocity`, `MjcfSensor` `dim`,
/// `needstage` and `datatype`, `MjcfSensorType` `Tendonactuatorfrc`, and
/// `MjcfConnect` `site1` and `site2`, its `body1` and `anchor` optional. 7 since a
/// flexcomp's unwritten `inertiabox` reads as MuJoCo's 0.005 and
/// one below mjMINVAL is refused (Rigid-loading PR RL-3b): a version-6 file
/// holds 0.0 there for a flexcomp that did not write it. 6 since `MjcfFlex`
/// took `vertex_bodies` (Rigid-loading L10).
pub const MJB_VERSION: u32 = 14;

/// Header size in bytes (magic + version + flags).
pub const MJB_HEADER_SIZE: usize = 12;

/// The most bytes a file's strings and byte buffers may claim, together, for
/// the readers to take it: 256 MiB.
pub const MJB_DECODE_LIMIT: usize = 1 << 28;

/// The readers' bincode configuration: the writers' with
/// [`MJB_DECODE_LIMIT`].
fn decode_config() -> impl bincode::config::Config {
    bincode::config::standard().with_limit::<MJB_DECODE_LIMIT>()
}

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
            return Err(MjcfError::MjbMagic { found: self.magic });
        }
        if self.version != MJB_VERSION {
            return Err(MjcfError::MjbVersion {
                found: self.version,
                supported: MJB_VERSION,
            });
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
/// - The model's body and frame tree is deeper than the loader takes
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
    let path = path.as_ref();
    let file = File::create(path).map_err(|e| MjcfError::io(path, e))?;
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
/// Returns an error if the model's body and frame tree is deeper than the
/// loader takes, if serialization fails, or if the writer does.
pub fn save_mjb_writer<W: Write>(model: &MjcfModel, writer: &mut W) -> Result<()> {
    let bytes = save_mjb_bytes(model)?;
    writer.write_all(&bytes).map_err(|e| MjcfError::MjbEncode {
        message: e.to_string(),
    })
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
/// Returns an error if the model's body and frame tree is deeper than the
/// loader takes, or if serialization fails.
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
    check_tree_depth(&model.worldbody)?;
    on_large_stack(|| {
        let encode_error = |message: String| MjcfError::MjbEncode { message };
        let mut buffer = Vec::new();
        MjbHeader::new()
            .write_to(&mut buffer)
            .map_err(|e| encode_error(e.to_string()))?;
        bincode::serde::encode_into_std_write(model, &mut buffer, bincode::config::standard())
            .map_err(|e| encode_error(e.to_string()))?;
        Ok(buffer)
    })
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
/// - Its strings and byte buffers claim more than [`MJB_DECODE_LIMIT`] bytes
/// - Its body and frame tree is deeper than the loader takes
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
    let path = path.as_ref();
    let file = File::open(path).map_err(|e| MjcfError::io(path, e))?;
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
/// - Its strings and byte buffers claim more than [`MJB_DECODE_LIMIT`] bytes
/// - Its body and frame tree is deeper than the loader takes
/// - Deserialization fails
pub fn load_mjb_reader<R: Read + Send>(reader: &mut R) -> Result<MjcfModel> {
    // Read and validate header
    let header = MjbHeader::read_from(reader).map_err(|e| MjcfError::MjbDecode {
        message: format!("failed to read header: {e}"),
    })?;
    header.validate()?;

    on_large_stack(|| {
        let (decoded, too_deep) =
            decoding(|| bincode::serde::decode_from_std_read(reader, decode_config()));
        decoded.map_err(|e| match e {
            _ if too_deep => MjcfError::too_deep(MAX_BODY_TREE_DEPTH),
            bincode::error::DecodeError::LimitExceeded => MjcfError::MjbTooLarge {
                limit: MJB_DECODE_LIMIT,
            },
            e => MjcfError::MjbDecode {
                message: e.to_string(),
            },
        })
    })
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
/// - Their strings and byte buffers claim more than [`MJB_DECODE_LIMIT`] bytes
/// - Their body and frame tree is deeper than the loader takes
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
                <compiler angle="radian"/>
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
        assert!(matches!(header.validate(), Err(MjcfError::MjbMagic { .. })));

        // Invalid version
        header.magic = MJB_MAGIC;
        header.version = 999;
        assert!(matches!(
            header.validate(),
            Err(MjcfError::MjbVersion { found: 999, .. })
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
    fn a_model_read_back_keeps_its_elements_order() {
        // `items` records a body's and a frame's elements in the document's
        // order, which a model read back keeps.
        let original = parse_mjcf_str(
            r#"<mujoco><worldbody><site name="a"/><frame><site name="f"/><composite type="cable" prefix="c" count="4 1 1" curve="s" size="1"><geom type="capsule" size=".01"/></composite></frame><body name="b"><geom name="g" size="1"/><frame><geom name="h" size="1"/></frame><site name="s"/></body></worldbody></mujoco>"#,
        )
        .expect("should parse");
        assert!(!original.worldbody.items.is_empty());
        let bytes = save_mjb_bytes(&original).expect("should serialize");
        let loaded = load_mjb_bytes(&bytes).expect("should deserialize");
        assert_eq!(loaded, original);
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
                <compiler angle="radian"/>
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
        let _mjb_size = bytes.len();
        let _xml_size = xml_bytes.len();
    }

    #[test]
    fn test_invalid_magic_bytes() {
        let bad_data = b"NOPE1234567890";
        let result = load_mjb_bytes(bad_data);
        assert!(matches!(result, Err(MjcfError::MjbMagic { .. })));
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
        assert!(matches!(
            result,
            Err(MjcfError::MjbVersion { found: 999, .. })
        ));
    }

    /// A file written before `MjcfFlex`'s shape changed (version 5) is
    /// refused for its version, not misread: decoded as version 6 it failed
    /// with a decode error (measured by RL-3a's review).
    #[test]
    fn a_version_5_file_is_refused_for_its_version() {
        let mut bytes = save_mjb_bytes(&simple_model()).expect("should serialize");
        bytes[4..8].copy_from_slice(&5u32.to_le_bytes());
        assert!(matches!(
            load_mjb_bytes(&bytes),
            Err(MjcfError::MjbVersion { found: 5, .. })
        ));
    }

    /// A version-6 file is refused for its version: it holds 0.0 as the
    /// `inertiabox` of a flexcomp that did not write one, which decoded as
    /// version 7 is refused as below mjMINVAL (measured on a file written
    /// at `6c2b5f5c`).
    #[test]
    fn a_version_6_file_is_refused_for_its_version() {
        let mut bytes = save_mjb_bytes(&simple_model()).expect("should serialize");
        bytes[4..8].copy_from_slice(&6u32.to_le_bytes());
        assert!(matches!(
            load_mjb_bytes(&bytes),
            Err(MjcfError::MjbVersion { found: 6, .. })
        ));
    }

    /// A version-7 file is refused for its version: its actuators and their
    /// defaults hold no `inheritrange`.
    #[test]
    fn a_version_7_file_is_refused_for_its_version() {
        let mut bytes = save_mjb_bytes(&simple_model()).expect("should serialize");
        bytes[4..8].copy_from_slice(&7u32.to_le_bytes());
        assert!(matches!(
            load_mjb_bytes(&bytes),
            Err(MjcfError::MjbVersion { found: 7, .. })
        ));
    }

    /// A version-8 file is refused for its version: its bodies hold no
    /// `xyaxes` or `zaxis`.
    #[test]
    fn a_version_8_file_is_refused_for_its_version() {
        let mut bytes = save_mjb_bytes(&simple_model()).expect("should serialize");
        bytes[4..8].copy_from_slice(&8u32.to_le_bytes());
        assert!(matches!(
            load_mjb_bytes(&bytes),
            Err(MjcfError::MjbVersion { found: 8, .. })
        ));
    }

    #[test]
    fn test_truncated_data() {
        let model = simple_model();
        let bytes = save_mjb_bytes(&model).expect("should serialize");

        // Truncate to just the header
        let truncated = &bytes[..MJB_HEADER_SIZE + 10];
        let result = load_mjb_bytes(truncated);
        assert!(matches!(result, Err(MjcfError::MjbDecode { .. })));
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

    /// A model whose bodies form a chain `levels` deep below the world, built
    /// without recursing.
    fn chain(levels: usize) -> MjcfModel {
        let mut body = crate::types::MjcfBody::default();
        for _ in 1..levels {
            let mut holder = crate::types::MjcfBody::default();
            holder.children.push(body);
            body = holder;
        }
        let mut model = MjcfModel::default();
        model.worldbody.children.push(body);
        model
    }

    /// How many levels of bodies a [`chain`] holds below the world.
    fn levels(model: &MjcfModel) -> usize {
        let mut levels = 0;
        let mut bodies = &model.worldbody.children;
        while let Some(body) = bodies.first() {
            levels += 1;
            bodies = &body.children;
        }
        levels
    }

    /// `f` on a thread of 256 KiB, too little to encode or decode a tree as
    /// deep as the loader takes (at main `b0590d0c` encoding 4096 levels
    /// overflowed 512 KiB, decoding them 8 MiB): what passes here ran on the
    /// loader's stack. A deep model is built, and dropped, on the caller's.
    fn on_small_stack<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
        std::thread::Builder::new()
            .stack_size(256 << 10)
            .spawn(f)
            .unwrap()
            .join()
            .unwrap()
    }

    /// The header, then a model name whose length claims `claim` bytes, and no
    /// more: bincode's varint, 252 then a u32 or 253 then a u64.
    fn claiming(claim: u64) -> Vec<u8> {
        let mut bytes = Vec::new();
        MjbHeader::new().write_to(&mut bytes).unwrap();
        if let Ok(short) = u32::try_from(claim) {
            bytes.push(252);
            bytes.extend_from_slice(&short.to_le_bytes());
        } else {
            bytes.push(253);
            bytes.extend_from_slice(&claim.to_le_bytes());
        }
        bytes
    }

    /// A file of a [`chain`] `levels` deep, encoded without the writers' depth
    /// check.
    fn deep_file(levels: usize) -> Vec<u8> {
        on_large_stack(move || {
            let mut bytes = Vec::new();
            MjbHeader::new().write_to(&mut bytes).unwrap();
            bincode::serde::encode_into_std_write(
                chain(levels),
                &mut bytes,
                bincode::config::standard(),
            )
            .unwrap();
            bytes
        })
    }

    #[test]
    fn a_length_past_the_limit_is_refused() {
        let limit = u64::try_from(MJB_DECODE_LIMIT).unwrap();
        assert!(matches!(
            load_mjb_bytes(&claiming(limit + 1)),
            Err(MjcfError::MjbTooLarge { limit }) if limit == MJB_DECODE_LIMIT
        ));
    }

    /// Not run at its parent: there the reader allocates the claimed length
    /// and the process aborts (measured at main `b0590d0c`).
    #[test]
    fn a_length_no_allocation_can_hold_is_refused_before_allocating() {
        assert!(matches!(
            load_mjb_bytes(&claiming(1 << 60)),
            Err(MjcfError::MjbTooLarge { .. })
        ));
    }

    /// Not run at its parent: there it overflows the stack (measured at main
    /// `b0590d0c`, see [`on_small_stack`]).
    #[test]
    fn a_tree_as_deep_as_the_loader_takes_round_trips_on_a_small_stack() {
        let model = std::sync::Arc::new(chain(MAX_BODY_TREE_DEPTH));
        let sent = std::sync::Arc::clone(&model);
        let read = on_small_stack(move || load_mjb_bytes(&save_mjb_bytes(&sent).unwrap()).unwrap());
        assert_eq!(levels(&read), MAX_BODY_TREE_DEPTH);
    }

    #[test]
    fn a_tree_deeper_than_the_loader_takes_is_not_written() {
        let model = std::sync::Arc::new(chain(MAX_BODY_TREE_DEPTH + 1));
        let sent = std::sync::Arc::clone(&model);
        let refused = on_small_stack(move || save_mjb_bytes(&sent).err());
        assert!(
            matches!(refused, Some(MjcfError::TooDeep { limit, .. }) if limit == MAX_BODY_TREE_DEPTH),
            "{refused:?}"
        );
    }

    /// Not run at its parent: there decoding either overflows the stack
    /// (measured at main `b0590d0c`).
    #[test]
    fn a_file_deeper_than_the_loader_takes_is_refused_without_overflowing() {
        for levels in [MAX_BODY_TREE_DEPTH + 1, 100_000] {
            let bytes = deep_file(levels);
            let refused = on_small_stack(move || load_mjb_bytes(&bytes).err());
            assert!(
                matches!(refused, Some(MjcfError::TooDeep { limit, .. }) if limit == MAX_BODY_TREE_DEPTH),
                "{levels}: {refused:?}"
            );
        }
    }

    /// Levels are counted down the tree, not across it: siblings share one.
    #[test]
    fn a_wide_tree_round_trips() {
        let mut model = MjcfModel::default();
        model.worldbody.children = (0..2 * MAX_BODY_TREE_DEPTH)
            .map(|_| crate::types::MjcfBody::default())
            .collect();
        let read = load_mjb_bytes(&save_mjb_bytes(&model).unwrap()).unwrap();
        assert_eq!(read.worldbody.children.len(), 2 * MAX_BODY_TREE_DEPTH);
    }

    /// Whether a decode met a level too deep is its own: on a thread that has
    /// decoded one (as the readers' thread is the caller's where the loader
    /// thread cannot start), a decode that fails for another reason is not
    /// reported too deep.
    #[test]
    fn a_decode_reports_only_its_own_depth() {
        let deep = deep_file(MAX_BODY_TREE_DEPTH + 1);
        let whole = save_mjb_bytes(&simple_model()).unwrap();
        let (first, second) = on_large_stack(move || {
            let decode = |payload: &[u8]| {
                decoding(|| {
                    bincode::serde::decode_from_slice::<MjcfModel, _>(payload, decode_config())
                        .is_err()
                })
            };
            let first = decode(&deep[MJB_HEADER_SIZE..]);
            let second = decode(&whole[MJB_HEADER_SIZE..whole.len() - 1]);
            (first, second)
        });
        assert_eq!(first, (true, true));
        assert_eq!(second, (true, false));
    }
}
