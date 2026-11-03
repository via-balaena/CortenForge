//! URDF XML parser.
//!
//! Reads a URDF document as MuJoCo 3.5.0's importer reads it (`xml_urdf.cc`,
//! `mjXURDF::Parse`): the document is read into a tree of elements, then in
//! the importer's passes — every named material's colour (`MakeMaterials`),
//! every link's name, every joint's parent and child links, then each link's
//! contents, then each joint's — so that of two faults it refuses the one
//! MuJoCo meets first. A sub-element the importer looks up by name
//! (`FindSubElem`) is refused when written twice, and when missing where it
//! is required; every number is read by MuJoCo's rules ([`read_numbers`]).
//! What the importer does not read is not read here: a visual's origin and
//! geometry (it discards visuals under URDF's `discardvisual`), a fixed
//! joint's limit and dynamics, a geometry's shapes after the one it takes, a
//! `velocity`. A `<mimic>`, which it does not read either, this crate makes
//! a joint equality of (`D-URDF-MIMIC`).

use std::collections::HashSet;

use nalgebra::Vector3;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};

use crate::error::{Result, UrdfError};
use crate::types::{
    UrdfCollision, UrdfGeometry, UrdfInertia, UrdfInertial, UrdfJoint, UrdfJointDynamics,
    UrdfJointLimit, UrdfJointType, UrdfLink, UrdfMimic, UrdfOrigin, UrdfRobot, UrdfVisual,
};

/// Parse a URDF string into a robot model.
///
/// # Errors
///
/// Returns an error if the XML is malformed, or where MuJoCo 3.5.0's importer
/// refuses the document.
pub fn parse_urdf_str(xml: &str) -> Result<UrdfRobot> {
    let root = read_tree(xml)?;
    if root.tag != "robot" {
        return Err(UrdfError::missing_element("robot", "URDF document"));
    }
    // MuJoCo's own sections in a URDF (`<mujoco><compiler|option|size>`) are
    // not read (ledger L378); the element is looked up as MuJoCo looks it up.
    root.find_sub("mujoco", false)?;
    let mut robot = UrdfRobot::new(root.attr("name").unwrap_or(""));
    make_materials(&root)?;

    let links: Vec<&Element> = root.children.iter().filter(|e| e.tag == "link").collect();
    let joints: Vec<&Element> = root.children.iter().filter(|e| e.tag == "joint").collect();
    let mut names = HashSet::new();
    for link in &links {
        let name = link.required_text("name")?;
        if !names.insert(name) {
            return Err(UrdfError::DuplicateLink(name.to_owned()));
        }
    }
    let mut children = HashSet::new();
    let mut edges = Vec::new();
    for joint in &joints {
        let joint_name = joint.attr("name").unwrap_or("");
        let parent = joint.required_sub("parent")?.required_text("link")?;
        let child = joint.required_sub("child")?.required_text("link")?;
        if let Some(missing) = [parent, child].into_iter().find(|l| !names.contains(l)) {
            return Err(UrdfError::undefined_link(missing, joint_name));
        }
        if !children.insert(child) {
            return Err(UrdfError::KinematicLoop(format!(
                "link '{child}' has multiple parent joints"
            )));
        }
        edges.push((parent, child));
    }
    // The links a root reaches (`AddToTree`): MuJoCo reads a link no root
    // reaches as "URDF body not found", at that link's turn below.
    let mut reached: HashSet<&str> = names
        .iter()
        .copied()
        .filter(|l| !children.contains(l))
        .collect();
    let mut frontier: Vec<&str> = reached.iter().copied().collect();
    while let Some(link) = frontier.pop() {
        for &(_, child) in edges.iter().filter(|(parent, _)| *parent == link) {
            if reached.insert(child) {
                frontier.push(child);
            }
        }
    }

    for link in links {
        let name = link.required_text("name")?;
        if !reached.contains(name) {
            return Err(UrdfError::KinematicLoop(format!(
                "link '{name}' is reached from no root link: its parent joints lead into a loop"
            )));
        }
        robot.links.push(parse_link(link)?);
    }
    for joint in joints {
        robot.joints.push(parse_joint(joint)?);
    }
    Ok(robot)
}

// ============================================================================
// The element tree
// ============================================================================

/// An element of the document: its tag, its attributes as tinyxml2 gives
/// them to MuJoCo (entities and character references replaced), its child
/// elements in order. Text is not kept: the importer reads none.
struct Element {
    tag: String,
    attrs: Vec<(String, String)>,
    children: Vec<Self>,
}

impl Element {
    /// A new element from its start tag. An attribute that is malformed or
    /// written twice is an XML error, as tinyxml2's is in MuJoCo.
    fn new(start: &BytesStart) -> Result<Self> {
        let tag = String::from_utf8_lossy(start.name().as_ref()).into_owned();
        let mut attrs = Vec::new();
        for attr in start.attributes() {
            let attr = attr.map_err(|e| UrdfError::XmlParse(e.to_string()))?;
            let raw =
                std::str::from_utf8(&attr.value).map_err(|e| UrdfError::XmlParse(e.to_string()))?;
            let value =
                quick_xml::escape::unescape(raw).map_err(|e| UrdfError::XmlParse(e.to_string()))?;
            attrs.push((
                String::from_utf8_lossy(attr.key.as_ref()).into_owned(),
                value.into_owned(),
            ));
        }
        Ok(Self {
            tag,
            attrs,
            children: Vec::new(),
        })
    }

    /// The attribute's text.
    fn attr(&self, name: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
    }

    /// The attribute's text, required (`ReadAttrTxt` with `required`).
    fn required_text(&self, name: &'static str) -> Result<&str> {
        self.attr(name)
            .ok_or_else(|| UrdfError::missing_attribute(name, &self.tag))
    }

    /// The child element `tag`, as MuJoCo's `FindSubElem` finds it
    /// (`xml_util.cc:901-926`): written twice, refused ("repeated element").
    fn find_sub(&self, tag: &'static str, required: bool) -> Result<Option<&Self>> {
        let mut found = None;
        for child in self.children.iter().filter(|c| c.tag == tag) {
            if found.is_some() {
                return Err(UrdfError::RepeatedElement {
                    element: tag,
                    context: self.tag.clone(),
                });
            }
            found = Some(child);
        }
        if required && found.is_none() {
            return Err(UrdfError::missing_element(tag, self.tag.clone()));
        }
        Ok(found)
    }

    /// The child element `tag`, required ([`Element::find_sub`]).
    fn required_sub(&self, tag: &'static str) -> Result<&Self> {
        self.find_sub(tag, true)?
            .ok_or_else(|| UrdfError::missing_element(tag, self.tag.clone()))
    }
}

/// The document's root element, read without recursing.
fn read_tree(xml: &str) -> Result<Element> {
    let mut reader = Reader::from_str(xml);
    let mut stack: Vec<Element> = Vec::new();
    let mut root = None;
    let mut buf = Vec::new();
    loop {
        let event = reader
            .read_event_into(&mut buf)
            .map_err(|e| UrdfError::XmlParse(e.to_string()))?;
        match event {
            Event::Start(ref e) => stack.push(Element::new(e)?),
            Event::Empty(ref e) => {
                let element = Element::new(e)?;
                match stack.last_mut() {
                    Some(parent) => parent.children.push(element),
                    None => root = root.or(Some(element)),
                }
            }
            Event::End(_) => {
                let element = stack
                    .pop()
                    .ok_or_else(|| UrdfError::XmlParse("unmatched end tag".into()))?;
                match stack.last_mut() {
                    Some(parent) => parent.children.push(element),
                    None => root = root.or(Some(element)),
                }
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }
    if !stack.is_empty() {
        return Err(UrdfError::XmlParse("unexpected end of the document".into()));
    }
    root.ok_or_else(|| UrdfError::missing_element("robot", "URDF document"))
}

// ============================================================================
// The importer's passes
// ============================================================================

/// Every named material's colour, as MuJoCo 3.5.0's `MakeMaterials` reads
/// them (`xml_urdf.cc:741-768`): every `<material>` in the document, at any
/// depth, in document order; one whose name no earlier material with a
/// colour registered has its `<color>`'s `rgba` read, four single-precision
/// numbers, required, and registers the name.
fn make_materials(root: &Element) -> Result<()> {
    let mut registered = HashSet::new();
    let mut stack = vec![root];
    while let Some(element) = stack.pop() {
        if element.tag == "material"
            && let Some(name) = element.attr("name")
            && !registered.contains(name)
            && let Some(color) = element.find_sub("color", false)?
        {
            read_floats::<4>(color, "rgba", true)?;
            registered.insert(name);
        }
        stack.extend(element.children.iter().rev());
    }
    Ok(())
}

/// A link's contents (`mjXURDF::Body`, `xml_urdf.cc:230-361`): its
/// `<inertial>`, then its visuals and collisions in order.
fn parse_link(element: &Element) -> Result<UrdfLink> {
    let mut link = UrdfLink::new(element.required_text("name")?);
    if let Some(inertial) = element.find_sub("inertial", false)? {
        link.inertial = Some(parse_inertial(inertial)?);
    }
    for child in &element.children {
        match child.tag.as_str() {
            "visual" => {
                if let Some(visual) = parse_visual(child)? {
                    link.visuals.push(visual);
                }
            }
            "collision" => link.collisions.push(parse_collision(child)?),
            _ => {}
        }
    }
    Ok(link)
}

/// An `<inertial>`: its origin, then its `<mass>` and `<inertia>`, required
/// with their values (`xml_urdf.cc:242-258`; measured: "missing element:
/// 'mass'", "required attribute missing: 'ixy'").
fn parse_inertial(element: &Element) -> Result<UrdfInertial> {
    let origin = parse_origin(element)?;
    let mass = required_number(element.required_sub("mass")?, "value")?;
    let inertia = element.required_sub("inertia")?;
    Ok(UrdfInertial {
        origin,
        mass,
        inertia: UrdfInertia {
            ixx: required_number(inertia, "ixx")?,
            iyy: required_number(inertia, "iyy")?,
            izz: required_number(inertia, "izz")?,
            ixy: required_number(inertia, "ixy")?,
            ixz: required_number(inertia, "ixz")?,
            iyz: required_number(inertia, "iyz")?,
        },
    })
}

/// An element's `<origin>` (`mjXURDF::Origin`, `xml_urdf.cc:650-670`): its
/// `xyz` and `rpy`, three numbers each where written; none, the identity.
fn parse_origin(element: &Element) -> Result<UrdfOrigin> {
    let Some(origin) = element.find_sub("origin", false)? else {
        return Ok(UrdfOrigin::default());
    };
    let xyz = read_vector3(origin, "xyz", false)?.unwrap_or_else(Vector3::zeros);
    let rpy = read_vector3(origin, "rpy", false)?.unwrap_or_else(Vector3::zeros);
    Ok(UrdfOrigin::new(xyz, rpy))
}

/// A `<visual>`, as MuJoCo 3.5.0's importer reads one under URDF's
/// `discardvisual` (`xml_urdf.cc:297-337`; measured): its `<material>` is
/// read — a `<color>`'s `rgba`, four numbers, required, else the material's
/// `name` — and its origin and geometry are not, so a visual whose origin or
/// geometry is not readable is dropped (`None`), where MuJoCo makes no geom.
fn parse_visual(element: &Element) -> Result<Option<UrdfVisual>> {
    let material = match element.find_sub("material", false)? {
        Some(m) => {
            if let Some(color) = m.find_sub("color", false)? {
                read_floats::<4>(color, "rgba", true)?;
            } else {
                m.required_text("name")?;
            }
            m.attr("name").map(str::to_owned)
        }
        None => None,
    };
    let origin = parse_origin(element).ok();
    let geometry = element
        .find_sub("geometry", true)
        .ok()
        .flatten()
        .and_then(|g| parse_geometry(g).ok());
    Ok(origin.zip(geometry).map(|(origin, geometry)| UrdfVisual {
        name: element.attr("name").map(str::to_owned),
        origin,
        geometry,
        material,
    }))
}

/// A `<collision>` (`mjXURDF::Geom`, `xml_urdf.cc:523-648`): its required
/// `<geometry>`, then its origin.
fn parse_collision(element: &Element) -> Result<UrdfCollision> {
    let geometry = parse_geometry(element.required_sub("geometry")?)?;
    Ok(UrdfCollision {
        name: element.attr("name").map(str::to_owned),
        origin: parse_origin(element)?,
        geometry,
    })
}

/// A `<geometry>`'s shape, the first of `box`, `cylinder`, `sphere`,
/// `capsule` and `mesh` it holds, in that order, the others not read
/// (`xml_urdf.cc:546-636`); each shape's numbers required; none, refused
/// (MuJoCo: "visual geometry specification not found").
fn parse_geometry(element: &Element) -> Result<UrdfGeometry> {
    if let Some(shape) = element.find_sub("box", false)? {
        return Ok(UrdfGeometry::Box {
            size: required_vector3(shape, "size")?,
        });
    }
    if let Some(shape) = element.find_sub("cylinder", false)? {
        return Ok(UrdfGeometry::Cylinder {
            radius: required_number(shape, "radius")?,
            length: required_number(shape, "length")?,
        });
    }
    if let Some(shape) = element.find_sub("sphere", false)? {
        return Ok(UrdfGeometry::Sphere {
            radius: required_number(shape, "radius")?,
        });
    }
    if let Some(shape) = element.find_sub("capsule", false)? {
        return Ok(UrdfGeometry::Capsule {
            radius: required_number(shape, "radius")?,
            length: required_number(shape, "length")?,
        });
    }
    if let Some(shape) = element.find_sub("mesh", false)? {
        return Ok(UrdfGeometry::Mesh {
            filename: shape.required_text("filename")?.to_owned(),
            scale: read_scale(shape)?,
        });
    }
    Err(UrdfError::missing_element("shape", "geometry"))
}

/// A joint's contents (`mjXURDF::Joint`, `xml_urdf.cc:375-519`): its type,
/// name, parent and child links, origin and axis — (1, 0, 0) without an
/// `<axis>`, whose `xyz` is required (`:411-414`; measured) — then, but for a
/// fixed joint, which MuJoCo returns before reading them, its `<dynamics>`
/// and `<limit>`. `velocity` MuJoCo does not read (measured: `velocity="x"`
/// loads): 0 here where it is not one number.
fn parse_joint(element: &Element) -> Result<UrdfJoint> {
    let type_str = element.required_text("type")?;
    let joint_type = UrdfJointType::from_str(type_str)
        .ok_or_else(|| UrdfError::UnknownJointType(type_str.to_owned()))?;
    let name = element.required_text("name")?;
    let parent = element.required_sub("parent")?.required_text("link")?;
    let child = element.required_sub("child")?.required_text("link")?;
    let origin = parse_origin(element)?;
    let axis = match element.find_sub("axis", false)? {
        Some(axis) => required_vector3(axis, "xyz")?,
        None => Vector3::x(),
    };
    let mut joint = UrdfJoint::new(name, joint_type, parent, child)
        .with_origin(origin)
        .with_axis(axis);
    if joint_type == UrdfJointType::Fixed {
        return Ok(joint);
    }
    if let Some(dynamics) = element.find_sub("dynamics", false)? {
        joint = joint.with_dynamics(UrdfJointDynamics {
            damping: read_number(dynamics, "damping", false)?.unwrap_or(0.0),
            friction: read_number(dynamics, "friction", false)?.unwrap_or(0.0),
        });
    }
    if let Some(limit) = element.find_sub("limit", false)? {
        joint = joint.with_limit(UrdfJointLimit {
            lower: read_number(limit, "lower", false)?,
            upper: read_number(limit, "upper", false)?,
            effort: read_number(limit, "effort", false)?.unwrap_or(0.0),
            velocity: limit
                .attr("velocity")
                .and_then(|s| s.trim().parse().ok())
                .unwrap_or(0.0),
        });
    }
    if let Some(mimic) = element.find_sub("mimic", false)? {
        joint = joint.with_mimic(UrdfMimic {
            joint: mimic.required_text("joint")?.to_owned(),
            multiplier: read_number(mimic, "multiplier", false)?.unwrap_or(1.0),
            offset: read_number(mimic, "offset", false)?.unwrap_or(0.0),
        });
    }
    Ok(joint)
}

// ============================================================================
// MuJoCo 3.5.0's attribute reader
// ============================================================================

/// C's `isspace` in the "C" locale: ASCII whitespace and the vertical tab.
fn is_c_space(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\n' | '\u{b}' | '\u{c}' | '\r')
}

/// Whether a token spells an infinity (`inf`, `-Infinity`), as Rust's parser
/// reads one, rather than overflowing to it.
fn names_infinity(token: &str) -> bool {
    token
        .trim_start_matches(['+', '-'])
        .get(..3)
        .is_some_and(|s| s.eq_ignore_ascii_case("inf"))
}

/// Whether a decimal token's mantissa has a nonzero digit, so that reading it
/// as zero or a subnormal is an underflow.
fn mantissa_is_nonzero(token: &str) -> bool {
    token
        .split(['e', 'E'])
        .next()
        .is_some_and(|m| m.bytes().any(|b| (b'1'..=b'9').contains(&b)))
}

/// Whether a token is a `NaN` with a payload, `nan(…)` in any case and with
/// a sign or none, which `strtod` reads as a `NaN` and Rust's parser refuses.
fn names_nan_with_payload(token: &str) -> bool {
    let unsigned = token.strip_prefix(['+', '-']).unwrap_or(token);
    unsigned
        .get(..4)
        .is_some_and(|nan| nan.eq_ignore_ascii_case("nan("))
        && unsigned[4..]
            .strip_suffix(')')
            .is_some_and(|payload| !payload.contains(')'))
}

/// One number of an attribute, as MuJoCo 3.5.0's `StringToVector` reads it
/// (`user_util.cc:1238-1255`), by sim-mjcf's reader's rules (`sim-mjcf`
/// `parser/attrs.rs`, `real_token`, which this crate cannot call: sim-mjcf
/// exports no reader, by choice, before 0.10 freezes its API;
/// `numbers_read_as_sim_mjcf_reads_them` holds the two to one table): a token
/// that is not a number is a bad format; a nonzero one that underflows to
/// zero or a subnormal, or one that overflows to an infinity, is MuJoCo's
/// "number is too large"; a written `NaN` or infinity, which MuJoCo loads,
/// is refused (`D-NONFINITE`), a `NaN` with a payload (`nan(1)`) included; a
/// hexadecimal float, which MuJoCo's `strtod` reads, is a bad format
/// (`D-HEX-FLOAT`). Where `single`, the number is read as a `float`, to its
/// range: an `rgba` (`xml_urdf.cc:304`, `:754`).
fn read_token(element: &str, name: &'static str, token: &str, single: bool) -> Result<f64> {
    token_value(token, single)
        .map_err(|fault| UrdfError::invalid_attribute(name, element, fault.words()))
}

/// Why a token is not read ([`token_value`]).
enum TokenFault {
    /// Not a number.
    BadFormat,
    /// It underflows or overflows its type.
    TooLarge,
    /// A written `NaN` or infinity.
    NonFinite,
}

impl TokenFault {
    /// MuJoCo's words for it (`xml_util.cc:708-716`), and this crate's for a
    /// non-finite number.
    fn words(&self) -> &'static str {
        match self {
            Self::BadFormat => "bad format",
            Self::TooLarge => "number is too large",
            Self::NonFinite => "a NaN or an infinity is not allowed",
        }
    }
}

/// A token's value by [`read_token`]'s rules.
fn token_value(token: &str, single: bool) -> std::result::Result<f64, TokenFault> {
    let Ok(x) = token.parse::<f64>() else {
        return Err(if names_nan_with_payload(token) {
            TokenFault::NonFinite
        } else {
            TokenFault::BadFormat
        });
    };
    let too_large = |x: f64, min_positive: f64| {
        (x.is_finite() && x.abs() < min_positive && mantissa_is_nonzero(token))
            || (x.is_infinite() && !names_infinity(token))
    };
    if too_large(x, f64::MIN_POSITIVE) {
        return Err(TokenFault::TooLarge);
    }
    if !x.is_finite() {
        return Err(TokenFault::NonFinite);
    }
    if single {
        let f = token.parse::<f32>().map_err(|_| TokenFault::BadFormat)?;
        if too_large(f64::from(f), f64::from(f32::MIN_POSITIVE)) {
            return Err(TokenFault::TooLarge);
        }
        return Ok(f64::from(f));
    }
    Ok(x)
}

/// `name`'s `N` numbers as MuJoCo 3.5.0's importer reads them (`ReadAttr`,
/// `exact`, `xml_util.cc:799-813`): absent, or written with no number, it is
/// not written (`None`), refused where `required` ("required attribute
/// missing"); fewer numbers than `N` do not have enough data, more have too
/// much.
fn read_numbers_as<const N: usize>(
    element: &Element,
    name: &'static str,
    required: bool,
    single: bool,
) -> Result<Option<[f64; N]>> {
    let values = match element.attr(name) {
        Some(text) => text
            .split(is_c_space)
            .filter(|t| !t.is_empty())
            .map(|t| read_token(&element.tag, name, t, single))
            .collect::<Result<Vec<_>>>()?,
        None => Vec::new(),
    };
    if values.is_empty() {
        return if required {
            Err(UrdfError::missing_attribute(name, &element.tag))
        } else {
            Ok(None)
        };
    }
    <[f64; N]>::try_from(values.as_slice())
        .map(Some)
        .map_err(|_| {
            let message = if values.len() < N {
                "does not have enough data"
            } else {
                "has too much data"
            };
            UrdfError::invalid_attribute(name, &element.tag, message)
        })
}

/// `name`'s `N` numbers ([`read_numbers_as`], doubles).
fn read_numbers<const N: usize>(
    element: &Element,
    name: &'static str,
    required: bool,
) -> Result<Option<[f64; N]>> {
    read_numbers_as::<N>(element, name, required, false)
}

/// `name`'s `N` single-precision numbers ([`read_numbers_as`]).
fn read_floats<const N: usize>(
    element: &Element,
    name: &'static str,
    required: bool,
) -> Result<Option<[f64; N]>> {
    read_numbers_as::<N>(element, name, required, true)
}

/// One number ([`read_numbers`]).
fn read_number(element: &Element, name: &'static str, required: bool) -> Result<Option<f64>> {
    Ok(read_numbers::<1>(element, name, required)?.map(|[x]| x))
}

/// Three numbers ([`read_numbers`]).
fn read_vector3(
    element: &Element,
    name: &'static str,
    required: bool,
) -> Result<Option<Vector3<f64>>> {
    Ok(read_numbers::<3>(element, name, required)?.map(Vector3::from))
}

/// A required number ([`read_number`]).
fn required_number(element: &Element, name: &'static str) -> Result<f64> {
    read_number(element, name, true)?
        .ok_or_else(|| UrdfError::missing_attribute(name, &element.tag))
}

/// A required three numbers ([`read_vector3`]).
fn required_vector3(element: &Element, name: &'static str) -> Result<Vector3<f64>> {
    read_vector3(element, name, true)?
        .ok_or_else(|| UrdfError::missing_attribute(name, &element.tag))
}

/// A mesh's `scale`, which MuJoCo 3.5.0's importer reads by `ReadAttrArr`
/// (`xml_urdf.cc:584`, `xml_util.h:126-147`, `xml_util.cc:596-636`), a
/// stream reader with words of its own (measured): written empty or blank, or
/// with a token it cannot read (an underflow and an overflow included),
/// "problem reading attribute"; more than three numbers "has too much data",
/// fewer "does not have enough data". Absent, `None`. Its tokens are read as
/// [`read_token`] reads them otherwise: a `NaN`, an infinity and a
/// hexadecimal float, which MuJoCo's stream reads, are refused here
/// (`D-NONFINITE`, `D-HEX-FLOAT`).
fn read_scale(element: &Element) -> Result<Option<Vector3<f64>>> {
    let name = "scale";
    let Some(text) = element.attr(name) else {
        return Ok(None);
    };
    let invalid = |message: &str| UrdfError::invalid_attribute(name, &element.tag, message);
    let tokens: Vec<&str> = text.split(is_c_space).filter(|t| !t.is_empty()).collect();
    if tokens.is_empty() {
        return Err(invalid("problem reading attribute"));
    }
    let mut values = Vec::new();
    for token in tokens.iter().take(3) {
        match token_value(token, false) {
            Ok(x) => values.push(x),
            Err(fault @ TokenFault::NonFinite) => return Err(invalid(fault.words())),
            Err(TokenFault::BadFormat | TokenFault::TooLarge) => {
                return Err(invalid("problem reading attribute"));
            }
        }
    }
    if tokens.len() > 3 {
        return Err(invalid("has too much data"));
    }
    if values.len() < 3 {
        return Err(invalid("does not have enough data"));
    }
    Ok(Some(Vector3::new(values[0], values[1], values[2])))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_parse_simple_robot() {
        let xml = r#"
            <robot name="test_robot">
                <link name="base_link">
                    <inertial>
                        <mass value="1.0"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                </link>
                <link name="link1"/>
            </robot>
        "#;

        let robot = parse_urdf_str(xml).expect("should parse");
        assert_eq!(robot.name, "test_robot");
        assert_eq!(robot.links.len(), 2);

        let base = robot.link("base_link").expect("base_link should exist");
        assert!(base.inertial.is_some());
        assert_relative_eq!(base.inertial.as_ref().map(|i| i.mass).unwrap_or(0.0), 1.0);
    }

    #[test]
    fn test_parse_joint() {
        let xml = r#"
            <robot name="test">
                <link name="base"/>
                <link name="child"/>
                <joint name="joint1" type="revolute">
                    <parent link="base"/>
                    <child link="child"/>
                    <axis xyz="0 0 1"/>
                    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
                </joint>
            </robot>
        "#;

        let robot = parse_urdf_str(xml).expect("should parse");
        assert_eq!(robot.joints.len(), 1);

        let joint = robot.joint("joint1").expect("joint1 should exist");
        assert_eq!(joint.joint_type, UrdfJointType::Revolute);
        assert_eq!(joint.parent, "base");
        assert_eq!(joint.child, "child");
        assert_relative_eq!(joint.axis.z, 1.0, epsilon = 1e-10);

        let limit = joint.limit.as_ref().expect("should have limit");
        assert_relative_eq!(limit.lower.expect("lower"), -1.57, epsilon = 1e-10);
        assert_relative_eq!(limit.upper.expect("upper"), 1.57, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_origin() {
        let xml = r#"
            <robot name="test">
                <link name="base">
                    <inertial>
                        <origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
                        <mass value="1"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                </link>
            </robot>
        "#;

        let robot = parse_urdf_str(xml).expect("should parse");
        let base = robot.link("base").expect("base should exist");
        let inertial = base.inertial.as_ref().expect("should have inertial");

        assert_relative_eq!(inertial.origin.xyz.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(inertial.origin.xyz.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(inertial.origin.xyz.z, 3.0, epsilon = 1e-10);
        assert_relative_eq!(inertial.origin.rpy.x, 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_collision_geometry() {
        let xml = r#"
            <robot name="test">
                <link name="base">
                    <collision>
                        <geometry>
                            <box size="1 2 3"/>
                        </geometry>
                    </collision>
                    <collision>
                        <geometry>
                            <sphere radius="0.5"/>
                        </geometry>
                    </collision>
                </link>
            </robot>
        "#;

        let robot = parse_urdf_str(xml).expect("should parse");
        let base = robot.link("base").expect("base should exist");
        assert_eq!(base.collisions.len(), 2);

        match &base.collisions[0].geometry {
            UrdfGeometry::Box { size } => {
                assert_relative_eq!(size.x, 1.0, epsilon = 1e-10);
                assert_relative_eq!(size.y, 2.0, epsilon = 1e-10);
                assert_relative_eq!(size.z, 3.0, epsilon = 1e-10);
            }
            _ => panic!("expected box geometry"),
        }

        match &base.collisions[1].geometry {
            UrdfGeometry::Sphere { radius } => {
                assert_relative_eq!(*radius, 0.5, epsilon = 1e-10);
            }
            _ => panic!("expected sphere geometry"),
        }
    }

    #[test]
    fn three_numbers_split_on_c_whitespace() {
        let e = Element {
            tag: "origin".into(),
            attrs: vec![("xyz".into(), "  1   2 \t 3 \u{b} ".into())],
            children: Vec::new(),
        };
        let v = read_vector3(&e, "xyz", false)
            .expect("reads")
            .expect("written");
        assert_eq!(
            [v.x, v.y, v.z].map(f64::to_bits),
            [1.0, 2.0, 3.0].map(f64::to_bits)
        );
    }

    const I6: &str = r#"ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0""#;

    /// A robot of one link `base` holding `inner`.
    fn one_link(inner: &str) -> String {
        format!(r#"<robot name="r"><link name="base">{inner}</link></robot>"#)
    }

    /// A robot of links `base` and `arm`, joined by a revolute `j1` holding
    /// `origin` and `extra`.
    fn two_links(origin: &str, extra: &str) -> String {
        format!(
            r#"<robot name="r"><link name="base"/><link name="arm"><inertial><mass value="1"/><inertia {I6}/></inertial></link><joint name="j1" type="revolute"><parent link="base"/><child link="arm"/>{origin}{extra}</joint></robot>"#
        )
    }

    /// Every document below MuJoCo 3.5.0's importer refuses (measured) but the
    /// two `NaN` rows, which it loads with a warning and this reader refuses
    /// (`D-NONFINITE`), for the attribute or element, or in MuJoCo's words,
    /// named; this reader refuses each. At `b8a6ffba` 23 of the 32 loaded, 8 were refused
    /// otherwise (a vector of other than three numbers, `1 2` and `nan(1)`
    /// as not a number, an empty `<geometry/>` read to the end of its link)
    /// and `mass empty` as here (run there).
    #[test]
    fn what_mujocos_importer_refuses_is_refused() {
        let joint = |limit: &str| format!(r#"<axis xyz="0 0 1"/>{limit}"#);
        let cases: Vec<(&str, String, &str)> = vec![
            ("lower bad", two_links("", &joint(r#"<limit lower="abc" upper="1"/>"#)), "lower"),
            ("lower two", two_links("", &joint(r#"<limit lower="-1 0" upper="1"/>"#)), "has too much data"),
            ("effort bad", two_links("", &joint(r#"<limit effort="x"/>"#)), "effort"),
            ("damping bad", two_links("", &joint(r#"<dynamics damping="x"/>"#)), "damping"),
            ("origin too much", two_links(r#"<origin xyz="0 0 1 4"/>"#, ""), "has too much data"),
            ("origin too little", two_links(r#"<origin xyz="0 1"/>"#, ""), "does not have enough data"),
            ("rpy bad", two_links(r#"<origin rpy="0 0 a"/>"#, ""), "rpy"),
            ("axis no xyz", two_links("", "<axis/>"), "xyz"),
            ("axis too much", two_links("", r#"<axis xyz="0 0 1 0"/>"#), "has too much data"),
            ("axis too little", two_links("", r#"<axis xyz="0 1"/>"#), "does not have enough data"),
            ("ixy missing", one_link(r#"<inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"/></inertial>"#), "ixy"),
            ("ixx bad", one_link(r#"<inertial><mass value="1"/><inertia ixx="a" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>"#), "ixx"),
            ("no inertia", one_link(r#"<inertial><mass value="1"/></inertial>"#), "inertia"),
            ("no mass", one_link(&format!("<inertial><inertia {I6}/></inertial>")), "mass"),
            ("empty inertial", one_link("<inertial/>"), "mass"),
            ("mass empty", one_link(&format!(r#"<inertial><mass value=""/><inertia {I6}/></inertial>"#)), "value"),
            ("mass two", one_link(&format!(r#"<inertial><mass value="1 2"/><inertia {I6}/></inertial>"#)), "has too much data"),
            ("mass underflow", one_link(&format!(r#"<inertial><mass value="1e-999"/><inertia {I6}/></inertial>"#)), "number is too large"),
            ("mass overflow", one_link(&format!(r#"<inertial><mass value="1e400"/><inertia {I6}/></inertial>"#)), "number is too large"),
            // MuJoCo loads a NaN with a warning; refused here (D-NONFINITE).
            (
                "mass nan payload",
                one_link(&format!(r#"<inertial><mass value="nan(1)"/><inertia {I6}/></inertial>"#)),
                "a NaN or an infinity is not allowed",
            ),
            ("mass nan", one_link(&format!(r#"<inertial><mass value="nan"/><inertia {I6}/></inertial>"#)), "a NaN or an infinity is not allowed"),
            ("sphere bad", one_link(r#"<collision><geometry><sphere radius="x"/></geometry></collision>"#), "radius"),
            ("sphere no radius", one_link("<collision><geometry><sphere/></geometry></collision>"), "radius"),
            ("box two", one_link(r#"<collision><geometry><box size="1 1"/></geometry></collision>"#), "does not have enough data"),
            ("cylinder no length", one_link(r#"<collision><geometry><cylinder radius="0.1"/></geometry></collision>"#), "length"),
            ("mesh scale two", one_link(r#"<collision><geometry><mesh filename="a.stl" scale="1 1"/></geometry></collision>"#), "does not have enough data"),
            ("collision no geometry", one_link(r#"<collision><origin xyz="0 0 0"/></collision>"#), "geometry"),
            ("collision no shape", one_link("<collision><geometry/></collision>"), "shape"),
            ("empty collision", one_link("<collision/>"), "geometry"),
            ("visual color three", one_link(r#"<visual><geometry><sphere radius="0.1"/></geometry><material name="m"><color rgba="1 0 0"/></material></visual>"#), "does not have enough data"),
            ("visual material unnamed", one_link(r#"<visual><geometry><sphere radius="0.1"/></geometry><material/></visual>"#), "name"),
            ("robot material color", r#"<robot name="r"><material name="m"><color rgba="1 0"/></material><link name="base"/></robot>"#.to_string(), "does not have enough data"),
        ];
        let mut missed = Vec::new();
        for (case, urdf, what) in &cases {
            match parse_urdf_str(urdf) {
                Err(e) if e.to_string().contains(what) => {}
                Err(e) => missed.push(format!("{case}: {e} (expected {what})")),
                Ok(_) => missed.push(format!("{case}: loads")),
            }
        }
        assert!(missed.is_empty(), "{}", missed.join("\n"));
    }

    /// What MuJoCo 3.5.0's importer does not read loads (measured): a
    /// `velocity`, a visual's origin and geometry (the visual dropped), and
    /// an unnamed robot-level material; an empty number list is the
    /// attribute not written (`b8a6ffba` refused `xyz=""`, run there).
    #[test]
    fn what_mujocos_importer_does_not_read_loads() {
        let cases = [
            ("velocity bad", two_links("", r#"<axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="x"/>"#)),
            ("visual geometry bad", one_link(r#"<visual><geometry><sphere radius="x"/></geometry></visual>"#)),
            ("visual no geometry", one_link(r#"<visual><origin xyz="0 0 0"/></visual>"#)),
            ("visual origin bad", one_link(r#"<visual><origin xyz="a"/><geometry><sphere radius="0.1"/></geometry></visual>"#)),
            ("origin empty", two_links(r#"<origin xyz=""/>"#, "")),
            ("unnamed robot material", r#"<robot name="r"><material><color rgba="1"/></material><link name="base"/></robot>"#.to_string()),
        ];
        for (case, urdf) in &cases {
            let robot = parse_urdf_str(urdf).unwrap_or_else(|e| panic!("{case}: {e}"));
            assert!(
                robot.links[0].visuals.is_empty() || !case.starts_with("visual"),
                "{case}"
            );
        }
    }

    /// A mesh's `scale` is read as MuJoCo 3.5.0's `ReadAttrArr` reads it
    /// (measured): empty, blank or unreadable, "problem reading attribute";
    /// too many or too few numbers in `ReadAttr`'s words. A hexadecimal float,
    /// which MuJoCo's stream reads, is refused (`D-HEX-FLOAT`).
    #[test]
    fn a_mesh_scale_is_read_as_mujocos_stream_reads_it() {
        let cases = [
            ("", "problem reading attribute"),
            ("   ", "problem reading attribute"),
            ("a 1 1", "problem reading attribute"),
            ("1e-310 1 1", "problem reading attribute"),
            ("1e400 1 1", "problem reading attribute"),
            ("1.5x 1 1", "problem reading attribute"),
            ("1 1 1 1", "has too much data"),
            ("1 1", "does not have enough data"),
            ("0x1p0 1 1", "problem reading attribute"),
        ];
        for (scale, words) in cases {
            let urdf = one_link(&format!(
                r#"<collision><geometry><mesh filename="a.stl" scale="{scale}"/></geometry></collision>"#
            ));
            let err = parse_urdf_str(&urdf).expect_err(scale);
            assert!(err.to_string().contains(words), "{scale:?}: {err}");
        }
        let urdf = one_link(
            r#"<collision><geometry><mesh filename="a.stl" scale=" +2 2 2 "/></geometry></collision>"#,
        );
        let robot = parse_urdf_str(&urdf).expect("parses");
        let UrdfGeometry::Mesh { scale, .. } = &robot.links[0].collisions[0].geometry else {
            panic!("a mesh");
        };
        assert_eq!(scale.map(|s| s.x.to_bits()), Some(2.0_f64.to_bits()));
    }

    /// A `<mimic>`'s numbers are read as every other (`D-URDF-MIMIC`: MuJoCo
    /// 3.5.0's importer does not read `<mimic>`, so it loads these).
    #[test]
    fn a_mimics_numbers_are_read_as_every_other() {
        for (attrs, words) in [
            (r#"joint="j1" multiplier="x""#, "bad format"),
            (r#"joint="j1" offset="1 2""#, "has too much data"),
            (r#"multiplier="2""#, "missing required attribute: joint"),
        ] {
            let urdf = format!(
                r#"<robot name="r"><link name="base"/><link name="a"/><link name="b"/><joint name="j1" type="revolute"><parent link="base"/><child link="a"/></joint><joint name="j2" type="revolute"><parent link="base"/><child link="b"/><mimic {attrs}/></joint></robot>"#
            );
            let err = parse_urdf_str(&urdf).expect_err(attrs);
            assert!(err.to_string().contains(words), "{attrs}: {err}");
        }
    }

    /// This reader's number rules are sim-mjcf's (`real_token`): each token
    /// below is accepted or refused alike as a URDF `<mass value>` and as an
    /// MJCF `<inertial mass>`.
    #[test]
    fn numbers_read_as_sim_mjcf_reads_them() {
        let tokens = [
            "1",
            " 1 ",
            "1.",
            ".5",
            "+1",
            "-0",
            "1e3",
            "1E-3",
            "1e-999",
            "1e-310",
            "1e400",
            "-1e400",
            "inf",
            "-Infinity",
            "nan",
            "NaN",
            "nan(1)",
            "0x1p1",
            "1e",
            "abc",
            "1,5",
            "1 2",
            "",
            "\u{b}2",
        ];
        for token in tokens {
            let urdf = one_link(&format!(
                r#"<inertial><mass value="{token}"/><inertia {I6}/></inertial>"#
            ));
            let mjcf = format!(
                r#"<mujoco><worldbody><body><inertial pos="0 0 0" mass="{token}" diaginertia="1 1 1"/></body></worldbody></mujoco>"#
            );
            let (ours, theirs) = (parse_urdf_str(&urdf), sim_mjcf::parse_mjcf_str(&mjcf));
            assert_eq!(
                ours.is_ok(),
                theirs.is_ok(),
                "{token:?}: {ours:?} vs {:?}",
                theirs.err()
            );
        }
    }

    /// A robot's name is optional, as MuJoCo 3.5.0's importer reads it
    /// (`xml_urdf.cc:115`; measured: `<robot>` without a name loads).
    #[test]
    fn a_robot_without_a_name_is_read() {
        let robot = parse_urdf_str(r#"<robot><link name="base"/></robot>"#).expect("parses");
        assert_eq!(robot.name, "");
        assert_eq!(robot.links.len(), 1);
    }

    #[test]
    fn test_unknown_joint_type() {
        let xml = r#"
            <robot name="test">
                <link name="a"/>
                <link name="b"/>
                <joint name="j" type="unknown">
                    <parent link="a"/>
                    <child link="b"/>
                </joint>
            </robot>
        "#;
        let result = parse_urdf_str(xml);
        assert!(matches!(result, Err(UrdfError::UnknownJointType(_))));
    }
}
