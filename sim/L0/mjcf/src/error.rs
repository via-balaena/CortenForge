//! Error types for MJCF parsing and loading.

use std::fmt;
use std::path::PathBuf;

use thiserror::Error;

/// Errors from reading MJCF and building a [`sim_core::Model`] from it.
///
/// Each message ends with where the problem was found, when that is known
/// ([`Location`]). The enum and each of its variants are `#[non_exhaustive]`:
/// match a variant with `{ .. }`.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum MjcfError {
    // ── reading input ──
    /// A file could not be read.
    #[error("cannot read '{}': {source}", path.display())]
    #[non_exhaustive]
    Io {
        /// The file, as the caller or the document named it.
        path: PathBuf,
        /// What the operating system reported.
        source: std::io::Error,
    },

    /// The input is not well-formed XML, or ends inside an element.
    #[error("XML syntax error: {message}{at}")]
    #[non_exhaustive]
    Xml {
        /// What the XML reader reported.
        message: String,
        /// Where it was found.
        at: Location,
    },

    /// An `<include>` could not be expanded.
    #[error("include error: {message}{at}")]
    #[non_exhaustive]
    Include {
        /// What went wrong.
        message: String,
        /// Where it was found.
        at: Location,
    },

    // ── elements and attributes ──
    /// A required attribute is missing.
    #[error("required attribute missing: '{attribute}' on <{element}>{at}")]
    #[non_exhaustive]
    MissingAttribute {
        /// The element's tag.
        element: String,
        /// The missing attribute.
        attribute: String,
        /// Where the element is.
        at: Location,
    },

    /// Attributes that exclude each other were set together.
    #[error("<{element}> may set only one of: {}{at}", attributes.join(", "))]
    #[non_exhaustive]
    ConflictingAttributes {
        /// The element's tag.
        element: String,
        /// The attributes, of which at most one may be set.
        attributes: Vec<String>,
        /// Where the element is.
        at: Location,
    },

    // ── attribute values ──
    /// A value that should be a number is not one.
    #[error("bad number \"{value}\"{at}")]
    #[non_exhaustive]
    BadNumber {
        /// The text that does not parse.
        value: String,
        /// Where it was found.
        at: Location,
    },

    /// An attribute has the wrong number of values.
    #[error("expected {expected} values, found {found}{at}")]
    #[non_exhaustive]
    WrongValueCount {
        /// How many values the attribute takes.
        expected: ValueCount,
        /// How many it has.
        found: usize,
        /// Where it was found.
        at: Location,
    },

    /// A keyword attribute has a value that is not one of its keywords.
    #[error("invalid keyword \"{value}\" in attribute '{attribute}' on <{element}>{at}")]
    #[non_exhaustive]
    InvalidKeyword {
        /// The element's tag.
        element: String,
        /// The attribute.
        attribute: String,
        /// The value given.
        value: String,
        /// Where the element is.
        at: Location,
    },

    /// A number is NaN or infinite.
    #[error("{attribute} on <{element}> is not finite ({value}){at}")]
    #[non_exhaustive]
    NonFinite {
        /// The element's tag.
        element: String,
        /// The attribute or field.
        attribute: String,
        /// The value given.
        value: f64,
        /// Where the element is.
        at: Location,
    },

    /// A value breaks one of MuJoCo's rules for its attribute.
    #[error("invalid {attribute} on <{element}>: {reason}{at}")]
    #[non_exhaustive]
    InvalidValue {
        /// The element's tag.
        element: String,
        /// The attribute or field.
        attribute: String,
        /// The rule it breaks.
        reason: String,
        /// Where the element is.
        at: Location,
    },

    // ── names, references, default classes ──
    /// Two elements of one kind share a name.
    #[error("repeated name '{name}' in {kind}{at}")]
    #[non_exhaustive]
    DuplicateName {
        /// The kind of element.
        kind: ElementKind,
        /// The repeated name.
        name: String,
        /// Where the second one is.
        at: Location,
    },

    /// A name refers to an element that does not exist.
    #[error("unknown {kind} '{name}'{at}")]
    #[non_exhaustive]
    UndefinedReference {
        /// The kind of element the name should refer to.
        kind: ElementKind,
        /// The name.
        name: String,
        /// Where the reference is.
        at: Location,
    },

    /// A `class` or `childclass` names no default class.
    #[error("unknown default class name '{name}'{at}")]
    #[non_exhaustive]
    UnknownClass {
        /// The class name.
        name: String,
        /// Where the reference is.
        at: Location,
    },

    // ── model-level compiler rules, assets ──
    /// The model breaks one of MuJoCo's compiler rules that is not about one
    /// attribute's value.
    #[error("{reason}{at}")]
    #[non_exhaustive]
    InvalidModel {
        /// The rule it breaks.
        reason: String,
        /// Where it was found.
        at: Location,
    },

    /// A mesh, height field or asset file could not be loaded or is not valid.
    #[error("{kind} '{name}': {reason}{at}")]
    #[non_exhaustive]
    Asset {
        /// The kind of asset.
        kind: ElementKind,
        /// The asset's name.
        name: String,
        /// What is wrong with it.
        reason: String,
        /// Where it was found.
        at: Location,
    },

    // ── stated limitation ──
    /// Valid MuJoCo 3.5.0 that this crate does not implement.
    #[error("not supported by sim-mjcf (valid in MuJoCo 3.5.0): {feature}{at}")]
    #[non_exhaustive]
    Unsupported {
        /// What is not supported.
        feature: String,
        /// Where it was found.
        at: Location,
    },

    // ── .mjb ──
    /// The bytes do not start with the MJB magic number.
    #[cfg(feature = "mjb")]
    #[error("not an MJB file: magic bytes {found:?}, expected \"MJB1\"")]
    #[non_exhaustive]
    MjbMagic {
        /// The first four bytes.
        found: [u8; 4],
    },

    /// The MJB file's version is not this crate's.
    #[cfg(feature = "mjb")]
    #[error("unsupported MJB version {found} (supported: {supported})")]
    #[non_exhaustive]
    MjbVersion {
        /// The file's version.
        found: u32,
        /// The version this crate reads and writes.
        supported: u32,
    },

    /// An MJB file could not be written.
    #[cfg(feature = "mjb")]
    #[error("MJB encode error: {message}")]
    #[non_exhaustive]
    MjbEncode {
        /// What the encoder reported.
        message: String,
    },

    /// An MJB file could not be read.
    #[cfg(feature = "mjb")]
    #[error("MJB decode error: {message}")]
    #[non_exhaustive]
    MjbDecode {
        /// What the decoder reported.
        message: String,
    },
}

/// Which kind of element an error is about. Displays as its MJCF tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ElementKind {
    /// `<body>`.
    Body,
    /// `<frame>`.
    Frame,
    /// `<joint>`.
    Joint,
    /// `<geom>`.
    Geom,
    /// `<site>`.
    Site,
    /// `<mesh>`.
    Mesh,
    /// `<hfield>`.
    Hfield,
    /// `<flex>` or `<flexcomp>`.
    Flex,
    /// `<pair>`.
    Pair,
    /// An `<equality>` constraint.
    Equality,
    /// A `<tendon>`.
    Tendon,
    /// An `<actuator>`.
    Actuator,
    /// A `<sensor>`.
    Sensor,
    /// `<key>`.
    Keyframe,
}

impl fmt::Display for ElementKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Body => "body",
            Self::Frame => "frame",
            Self::Joint => "joint",
            Self::Geom => "geom",
            Self::Site => "site",
            Self::Mesh => "mesh",
            Self::Hfield => "hfield",
            Self::Flex => "flex",
            Self::Pair => "pair",
            Self::Equality => "equality",
            Self::Tendon => "tendon",
            Self::Actuator => "actuator",
            Self::Sensor => "sensor",
            Self::Keyframe => "keyframe",
        })
    }
}

/// How many values an attribute takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValueCount {
    /// Exactly this many.
    Exactly(usize),
    /// This many or more (the reader takes the first ones).
    AtLeast(usize),
}

impl fmt::Display for ValueCount {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Exactly(n) => write!(f, "exactly {n}"),
            Self::AtLeast(n) => write!(f, "at least {n}"),
        }
    }
}

/// Where an error was found: empty, or a line and element path for an error
/// found while parsing, or the element for an error found while building.
///
/// Display-only. A line counts lines of the text that was parsed: for
/// [`crate::load_model_from_file`] that is the text after `<include>`s are
/// expanded, not the included file. A parse error raised after more of the
/// input was read than its element names the last element read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Location(Option<Box<Place>>);

#[derive(Debug, Clone, PartialEq, Eq)]
enum Place {
    Parsed {
        line: usize,
        path: String,
    },
    Element {
        kind: ElementKind,
        ident: Ident,
        body: Option<String>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Ident {
    Name(String),
    Index(usize),
    None,
}

impl Location {
    /// No location.
    pub(crate) fn unknown() -> Self {
        Self(None)
    }

    /// A line of parsed text and the path of elements to it (`worldbody >
    /// body 'a' > geom`); an empty path shows the line alone.
    pub(crate) fn parsed(line: usize, path: String) -> Self {
        Self(Some(Box::new(Place::Parsed { line, path })))
    }

    /// An element by its name, or by kind alone when the name is empty.
    pub(crate) fn named(kind: ElementKind, name: &str) -> Self {
        let ident = if name.is_empty() {
            Ident::None
        } else {
            Ident::Name(name.to_owned())
        };
        Self::element(kind, ident)
    }

    /// An element by its name, or by its index among its kind when it has
    /// none (`actuator #3`).
    pub(crate) fn nth(kind: ElementKind, name: Option<&str>, index: usize) -> Self {
        let ident = match name {
            Some(n) if !n.is_empty() => Ident::Name(n.to_owned()),
            _ => Ident::Index(index),
        };
        Self::element(kind, ident)
    }

    fn element(kind: ElementKind, ident: Ident) -> Self {
        Self(Some(Box::new(Place::Element {
            kind,
            ident,
            body: None,
        })))
    }

    /// The body the element belongs to.
    pub(crate) fn in_body(mut self, body: &str) -> Self {
        if let Some(place) = self.0.as_deref_mut()
            && let Place::Element { body: b, .. } = place
            && !body.is_empty()
        {
            *b = Some(body.to_owned());
        }
        self
    }

    fn is_unknown(&self) -> bool {
        self.0.is_none()
    }
}

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0.as_deref() {
            None => Ok(()),
            Some(Place::Parsed { line, path }) if path.is_empty() => write!(f, " (line {line})"),
            Some(Place::Parsed { line, path }) => write!(f, " (line {line}: {path})"),
            Some(Place::Element { kind, ident, body }) => {
                match ident {
                    Ident::Name(n) => write!(f, " ({kind} '{n}'")?,
                    Ident::Index(i) => write!(f, " ({kind} #{i}")?,
                    Ident::None => write!(f, " ({kind}")?,
                }
                if let Some(b) = body {
                    write!(f, " in body '{b}'")?;
                }
                f.write_str(")")
            }
        }
    }
}

// Constructors, each with an unknown location; `at` sets one.
impl MjcfError {
    /// An [`MjcfError::Io`].
    pub(crate) fn io(path: impl Into<PathBuf>, source: std::io::Error) -> Self {
        Self::Io {
            path: path.into(),
            source,
        }
    }

    /// An [`MjcfError::Xml`].
    pub(crate) fn xml(message: impl fmt::Display) -> Self {
        Self::Xml {
            message: message.to_string(),
            at: Location::unknown(),
        }
    }

    /// The input ended inside `<tag>`.
    pub(crate) fn eof_in(tag: impl fmt::Display) -> Self {
        Self::xml(format!("unexpected EOF in {tag}"))
    }

    /// An [`MjcfError::Include`].
    pub(crate) fn include(message: impl Into<String>) -> Self {
        Self::Include {
            message: message.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::MissingAttribute`].
    pub(crate) fn missing_attribute(
        element: impl Into<String>,
        attribute: impl Into<String>,
    ) -> Self {
        Self::MissingAttribute {
            element: element.into(),
            attribute: attribute.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::ConflictingAttributes`].
    pub(crate) fn conflicting(element: impl Into<String>, attributes: &[&str]) -> Self {
        Self::ConflictingAttributes {
            element: element.into(),
            attributes: attributes.iter().map(|a| (*a).to_owned()).collect(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::BadNumber`].
    pub(crate) fn bad_number(value: impl Into<String>) -> Self {
        Self::BadNumber {
            value: value.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::WrongValueCount`].
    pub(crate) fn wrong_count(expected: ValueCount, found: usize) -> Self {
        Self::WrongValueCount {
            expected,
            found,
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::InvalidKeyword`].
    pub(crate) fn invalid_keyword(
        element: impl Into<String>,
        attribute: impl Into<String>,
        value: impl Into<String>,
    ) -> Self {
        Self::InvalidKeyword {
            element: element.into(),
            attribute: attribute.into(),
            value: value.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::NonFinite`].
    pub(crate) fn non_finite(
        element: impl Into<String>,
        attribute: impl Into<String>,
        value: f64,
    ) -> Self {
        Self::NonFinite {
            element: element.into(),
            attribute: attribute.into(),
            value,
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::InvalidValue`].
    pub(crate) fn invalid_value(
        element: impl Into<String>,
        attribute: impl Into<String>,
        reason: impl Into<String>,
    ) -> Self {
        Self::InvalidValue {
            element: element.into(),
            attribute: attribute.into(),
            reason: reason.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::DuplicateName`].
    pub(crate) fn duplicate_name(kind: ElementKind, name: impl Into<String>) -> Self {
        Self::DuplicateName {
            kind,
            name: name.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::UndefinedReference`].
    pub(crate) fn undefined(kind: ElementKind, name: impl Into<String>) -> Self {
        Self::UndefinedReference {
            kind,
            name: name.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::UnknownClass`].
    pub(crate) fn unknown_class(name: impl Into<String>) -> Self {
        Self::UnknownClass {
            name: name.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::InvalidModel`].
    pub(crate) fn invalid_model(reason: impl Into<String>) -> Self {
        Self::InvalidModel {
            reason: reason.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::Asset`].
    pub(crate) fn asset(
        kind: ElementKind,
        name: impl Into<String>,
        reason: impl Into<String>,
    ) -> Self {
        Self::Asset {
            kind,
            name: name.into(),
            reason: reason.into(),
            at: Location::unknown(),
        }
    }

    /// An [`MjcfError::Unsupported`].
    pub(crate) fn unsupported(feature: impl Into<String>) -> Self {
        Self::Unsupported {
            feature: feature.into(),
            at: Location::unknown(),
        }
    }

    /// The error with its location set to `at` (variants without one are
    /// returned as they are).
    pub(crate) fn at(mut self, at: Location) -> Self {
        if let Some(slot) = self.location_mut() {
            *slot = at;
        }
        self
    }

    /// The error with its location set to `at` if it has none yet.
    pub(crate) fn with_location_if_unknown(mut self, at: Location) -> Self {
        if let Some(slot) = self.location_mut()
            && slot.is_unknown()
        {
            *slot = at;
        }
        self
    }

    fn location_mut(&mut self) -> Option<&mut Location> {
        match self {
            Self::Xml { at, .. }
            | Self::Include { at, .. }
            | Self::MissingAttribute { at, .. }
            | Self::ConflictingAttributes { at, .. }
            | Self::BadNumber { at, .. }
            | Self::WrongValueCount { at, .. }
            | Self::InvalidKeyword { at, .. }
            | Self::NonFinite { at, .. }
            | Self::InvalidValue { at, .. }
            | Self::DuplicateName { at, .. }
            | Self::UndefinedReference { at, .. }
            | Self::UnknownClass { at, .. }
            | Self::InvalidModel { at, .. }
            | Self::Asset { at, .. }
            | Self::Unsupported { at, .. } => Some(at),
            Self::Io { .. } => None,
            #[cfg(feature = "mjb")]
            Self::MjbMagic { .. }
            | Self::MjbVersion { .. }
            | Self::MjbEncode { .. }
            | Self::MjbDecode { .. } => None,
        }
    }
}

/// Result type for MJCF operations.
pub type Result<T> = std::result::Result<T, MjcfError>;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn a_location_names_the_element_its_body_or_its_line() {
        assert_eq!(Location::unknown().to_string(), "");
        assert_eq!(
            Location::named(ElementKind::Geom, "g")
                .in_body("arm")
                .to_string(),
            " (geom 'g' in body 'arm')"
        );
        assert_eq!(
            Location::nth(ElementKind::Actuator, Some(""), 0).to_string(),
            " (actuator #0)"
        );
        assert_eq!(
            Location::named(ElementKind::Site, "").to_string(),
            " (site)"
        );
        assert_eq!(
            Location::parsed(4, "worldbody > body 'a' > geom".into()).to_string(),
            " (line 4: worldbody > body 'a' > geom)"
        );
        assert_eq!(Location::parsed(2, String::new()).to_string(), " (line 2)");
    }

    #[test]
    fn a_location_is_kept_once_set() {
        let first = Location::named(ElementKind::Body, "a");
        let err = MjcfError::duplicate_name(ElementKind::Body, "a")
            .at(first.clone())
            .with_location_if_unknown(Location::parsed(9, String::new()));
        assert!(
            matches!(&err, MjcfError::DuplicateName { at, .. } if at == &first),
            "{err:?}"
        );
        assert_eq!(err.to_string(), "repeated name 'a' in body (body 'a')");
    }

    #[test]
    fn displays_name_what_is_wrong() {
        let cases = [
            (
                MjcfError::missing_attribute("pair", "geom1"),
                "required attribute missing: 'geom1' on <pair>",
            ),
            (
                MjcfError::conflicting("actuator", &["joint", "tendon"]),
                "<actuator> may set only one of: joint, tendon",
            ),
            (MjcfError::bad_number("x"), "bad number \"x\""),
            (
                MjcfError::wrong_count(ValueCount::AtLeast(3), 2),
                "expected at least 3 values, found 2",
            ),
            (
                MjcfError::invalid_keyword("compiler", "angle", "grad"),
                "invalid keyword \"grad\" in attribute 'angle' on <compiler>",
            ),
            (
                MjcfError::non_finite("key", "time", f64::INFINITY),
                "time on <key> is not finite (inf)",
            ),
            (
                MjcfError::undefined(ElementKind::Joint, "nope"),
                "unknown joint 'nope'",
            ),
            (
                MjcfError::unsupported("hfield files"),
                "not supported by sim-mjcf (valid in MuJoCo 3.5.0): hfield files",
            ),
        ];
        for (err, text) in cases {
            assert_eq!(err.to_string(), text);
        }
    }
}
