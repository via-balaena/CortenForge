//! URDF XML parser.
//!
//! Parses URDF XML into the intermediate representation types.

use nalgebra::Vector3;
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{Result, UrdfError};
use crate::types::{
    UrdfCollision, UrdfGeometry, UrdfInertia, UrdfInertial, UrdfJoint, UrdfJointDynamics,
    UrdfJointLimit, UrdfJointType, UrdfLink, UrdfOrigin, UrdfRobot, UrdfVisual,
};

/// Parse a URDF string into a robot model.
///
/// # Errors
///
/// Returns an error if the XML is malformed or missing required elements.
pub fn parse_urdf_str(xml: &str) -> Result<UrdfRobot> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    parse_urdf_reader(&mut reader)
}

/// Parse URDF from a reader.
fn parse_urdf_reader<R: BufRead>(reader: &mut Reader<R>) -> Result<UrdfRobot> {
    let mut buf = Vec::new();
    let mut robot: Option<UrdfRobot> = None;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"robot" => {
                robot = Some(parse_robot(reader, e)?);
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    robot.ok_or_else(|| UrdfError::missing_element("robot", "URDF document"))
}

/// Parse the robot element and its children.
fn parse_robot<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<UrdfRobot> {
    let name = get_attribute(start, "name")?;
    let mut robot = UrdfRobot::new(name);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"link" => {
                        let link = parse_link(reader, e)?;
                        robot.links.push(link);
                    }
                    b"joint" => {
                        let joint = parse_joint(reader, e)?;
                        robot.joints.push(joint);
                    }
                    b"material" => read_material(reader, e)?,
                    // Skip gazebo and other elements
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                // Handle self-closing elements
                if e.name().as_ref() == b"link" {
                    // Empty link (just a name, no inertial/visual/collision)
                    let name = get_attribute(e, "name")?;
                    robot.links.push(UrdfLink::new(name));
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"robot" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in robot".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(robot)
}

/// Parse a link element.
fn parse_link<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<UrdfLink> {
    let name = get_attribute(start, "name")?;
    let mut link = UrdfLink::new(name);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"inertial" => {
                        link.inertial = Some(parse_inertial(reader)?);
                    }
                    b"visual" => {
                        if let Some(visual) = parse_visual(reader, e)? {
                            link.visuals.push(visual);
                        }
                    }
                    b"collision" => link.collisions.push(parse_collision(reader, e)?),
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            // MuJoCo 3.5.0's importer reads an element whatever its form: an
            // empty `<inertial/>` has no `<mass>`, an empty `<collision/>` no
            // `<geometry>` (measured); an empty `<visual/>` it does not read.
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"inertial" => return Err(UrdfError::missing_element("mass", "inertial")),
                b"collision" => return Err(UrdfError::missing_element("geometry", "collision")),
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"link" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in link".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(link)
}

/// Parse an inertial element: its `<mass>` and `<inertia>` are required, as
/// MuJoCo 3.5.0's importer requires them ("missing element: 'mass'",
/// `xml_urdf.cc:248-252`; measured).
fn parse_inertial<R: BufRead>(reader: &mut Reader<R>) -> Result<UrdfInertial> {
    let mut inertial = UrdfInertial::default();
    let (mut mass, mut inertia) = (false, false);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"origin" => inertial.origin = parse_origin(e)?,
                b"mass" => {
                    inertial.mass = required_number(e, "value")?;
                    mass = true;
                }
                b"inertia" => {
                    inertial.inertia = parse_inertia_element(e)?;
                    inertia = true;
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"inertial" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in inertial".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    if !mass {
        return Err(UrdfError::missing_element("mass", "inertial"));
    }
    if !inertia {
        return Err(UrdfError::missing_element("inertia", "inertial"));
    }
    Ok(inertial)
}

/// Parse origin element attributes: `xyz` and `rpy`, three numbers each
/// where written.
fn parse_origin(e: &BytesStart) -> Result<UrdfOrigin> {
    let xyz = read_vector3(e, "xyz", false)?.unwrap_or_else(Vector3::zeros);
    let rpy = read_vector3(e, "rpy", false)?.unwrap_or_else(Vector3::zeros);
    Ok(UrdfOrigin::new(xyz, rpy))
}

/// Parse inertia element attributes: all six are required, as MuJoCo
/// 3.5.0's importer requires them (`xml_urdf.cc:253-258`; measured:
/// "required attribute missing: 'ixy'").
fn parse_inertia_element(e: &BytesStart) -> Result<UrdfInertia> {
    Ok(UrdfInertia {
        ixx: required_number(e, "ixx")?,
        iyy: required_number(e, "iyy")?,
        izz: required_number(e, "izz")?,
        ixy: required_number(e, "ixy")?,
        ixz: required_number(e, "ixz")?,
        iyz: required_number(e, "iyz")?,
    })
}

/// Parse a visual element, as MuJoCo 3.5.0's importer reads one under
/// URDF's `discardvisual` (`xml_urdf.cc:297-337`; measured): its
/// `<material>` is read ([`read_material_in`]), its origin and geometry are
/// not, so a visual whose origin or geometry is not readable is dropped
/// (`None`), where MuJoCo makes no geom of it.
fn parse_visual<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<Option<UrdfVisual>> {
    let name = get_attribute_opt(start, "name");
    let mut origin = Some(UrdfOrigin::default());
    let mut geometry: Option<UrdfGeometry> = None;
    let mut material: Option<String> = None;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => match e.name().as_ref() {
                b"origin" => origin = origin.and_then(|_| parse_origin(e).ok()),
                b"geometry" => geometry = parse_geometry(reader).ok(),
                b"material" => material = read_material_in(reader, e, false)?,
                _ => {}
            },
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"origin" => origin = origin.and_then(|_| parse_origin(e).ok()),
                b"material" => material = read_material_in(reader, e, true)?,
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"visual" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in visual".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(origin.zip(geometry).map(|(origin, geometry)| UrdfVisual {
        name,
        origin,
        geometry,
        material,
    }))
}

/// A visual's `<material>`, read as MuJoCo 3.5.0's importer reads it
/// (`xml_urdf.cc:301-315`): a `<color>`'s `rgba`, four numbers, required;
/// with no `<color>`, the material's `name` required. Returns the name.
/// `empty` is whether the element is self-closing (then it has no children).
fn read_material_in<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    empty: bool,
) -> Result<Option<String>> {
    let name = get_attribute_opt(start, "name");
    let color = if empty { false } else { read_color(reader)? };
    if !color && name.is_none() {
        return Err(UrdfError::missing_attribute("name", "material"));
    }
    Ok(name)
}

/// The rest of a `<material>`: whether it has a `<color>`, whose `rgba`,
/// four numbers, is required (`xml_urdf.cc:303-305`, `:753-754`).
fn read_color<R: BufRead>(reader: &mut Reader<R>) -> Result<bool> {
    let mut color = false;
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) if e.name().as_ref() == b"color" => {
                read_numbers::<4>(e, "rgba", true)?;
                color = true;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"material" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in material".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(color)
}

/// A robot-level `<material>`, read as MuJoCo 3.5.0's importer reads one
/// (`MakeMaterials`, `xml_urdf.cc:745-760`): a `<color>`'s `rgba`, four
/// numbers, required where the material is named; otherwise not read.
fn read_material<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<()> {
    if get_attribute_opt(start, "name").is_some() {
        read_color(reader)?;
    } else {
        skip_element(reader, b"material")?;
    }
    Ok(())
}

/// Parse a collision element.
fn parse_collision<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<UrdfCollision> {
    let name = get_attribute_opt(start, "name");
    let mut origin = UrdfOrigin::default();
    let mut geometry: Option<UrdfGeometry> = None;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => match e.name().as_ref() {
                b"origin" => origin = parse_origin(e)?,
                b"geometry" => geometry = Some(parse_geometry(reader)?),
                _ => {}
            },
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"origin" => origin = parse_origin(e)?,
                // An empty `<geometry/>` names no shape (measured: "visual
                // geometry specification not found").
                b"geometry" => return Err(UrdfError::missing_element("shape", "geometry")),
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"collision" => break,
            Ok(Event::Eof) => {
                return Err(UrdfError::XmlParse("unexpected EOF in collision".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    let geometry = geometry.ok_or_else(|| UrdfError::missing_element("geometry", "collision"))?;

    Ok(UrdfCollision {
        name,
        origin,
        geometry,
    })
}

/// Parse a geometry element.
fn parse_geometry<R: BufRead>(reader: &mut Reader<R>) -> Result<UrdfGeometry> {
    let mut buf = Vec::new();
    let mut geometry: Option<UrdfGeometry> = None;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                // Each shape's numbers required, as MuJoCo 3.5.0's importer
                // reads them (`xml_urdf.cc:546-584`).
                b"box" => {
                    let size = required_vector3(e, "size")?;
                    geometry = Some(UrdfGeometry::Box { size });
                }
                b"cylinder" => {
                    let radius = required_number(e, "radius")?;
                    let length = required_number(e, "length")?;
                    geometry = Some(UrdfGeometry::Cylinder { radius, length });
                }
                b"sphere" => {
                    let radius = required_number(e, "radius")?;
                    geometry = Some(UrdfGeometry::Sphere { radius });
                }
                b"mesh" => {
                    let filename = get_attribute(e, "filename")?;
                    let scale = read_vector3(e, "scale", false)?;
                    geometry = Some(UrdfGeometry::Mesh { filename, scale });
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"geometry" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in geometry".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    geometry.ok_or_else(|| UrdfError::missing_element("shape", "geometry"))
}

/// Parse a joint element.
fn parse_joint<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<UrdfJoint> {
    let name = get_attribute(start, "name")?;
    let type_str = get_attribute(start, "type")?;
    let joint_type =
        UrdfJointType::from_str(&type_str).ok_or_else(|| UrdfError::UnknownJointType(type_str))?;

    let mut parent: Option<String> = None;
    let mut child: Option<String> = None;
    let mut origin = UrdfOrigin::default();
    // Without an `<axis>`, (1, 0, 0) (`xml_urdf.cc:412`; measured).
    let mut axis = Vector3::x();
    let mut limit: Option<UrdfJointLimit> = None;
    let mut dynamics: Option<UrdfJointDynamics> = None;
    let mut mimic: Option<crate::types::UrdfMimic> = None;

    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"parent" => {
                        parent = Some(get_attribute(e, "link")?);
                    }
                    b"child" => {
                        child = Some(get_attribute(e, "link")?);
                    }
                    b"origin" => {
                        origin = parse_origin(e)?;
                    }
                    // A written `<axis>` requires its `xyz`
                    // (`xml_urdf.cc:413-414`; measured).
                    b"axis" => axis = required_vector3(e, "xyz")?,
                    b"limit" => {
                        limit = Some(parse_joint_limit(e)?);
                    }
                    b"dynamics" => {
                        dynamics = Some(parse_joint_dynamics(e)?);
                    }
                    // MuJoCo 3.5.0's importer does not read `<mimic>`
                    // (D-URDF-MIMIC); its numbers are read as every other.
                    b"mimic" => {
                        let leader = get_attribute(e, "joint")?;
                        let multiplier = read_number(e, "multiplier", false)?.unwrap_or(1.0);
                        let offset = read_number(e, "offset", false)?.unwrap_or(0.0);
                        mimic = Some(crate::types::UrdfMimic {
                            joint: leader,
                            multiplier,
                            offset,
                        });
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"joint" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in joint".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    let parent =
        parent.ok_or_else(|| UrdfError::missing_element("parent", format!("joint '{name}'")))?;
    let child =
        child.ok_or_else(|| UrdfError::missing_element("child", format!("joint '{name}'")))?;

    let mut joint = UrdfJoint::new(name, joint_type, parent, child)
        .with_origin(origin)
        .with_axis(axis);

    if let Some(l) = limit {
        joint = joint.with_limit(l);
    }
    if let Some(d) = dynamics {
        joint = joint.with_dynamics(d);
    }
    if let Some(m) = mimic {
        joint = joint.with_mimic(m);
    }

    Ok(joint)
}

/// Parse joint limit element: `lower`, `upper` and `effort` read as MuJoCo
/// 3.5.0's importer reads them (`xml_urdf.cc:500-516`). It does not read
/// `velocity` (measured: `velocity="x"` loads), so a `velocity` that is not
/// one number is 0 here.
fn parse_joint_limit(e: &BytesStart) -> Result<UrdfJointLimit> {
    Ok(UrdfJointLimit {
        lower: read_number(e, "lower", false)?,
        upper: read_number(e, "upper", false)?,
        effort: read_number(e, "effort", false)?.unwrap_or(0.0),
        velocity: get_attribute_opt(e, "velocity")
            .and_then(|s| s.trim().parse().ok())
            .unwrap_or(0.0),
    })
}

/// Parse joint dynamics element.
fn parse_joint_dynamics(e: &BytesStart) -> Result<UrdfJointDynamics> {
    Ok(UrdfJointDynamics {
        damping: read_number(e, "damping", false)?.unwrap_or(0.0),
        friction: read_number(e, "friction", false)?.unwrap_or(0.0),
    })
}

// ============================================================================
// Helper functions
// ============================================================================

/// Get a required attribute value.
fn get_attribute(e: &BytesStart, name: &'static str) -> Result<String> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == name.as_bytes() {
            return String::from_utf8(attr.value.to_vec())
                .map_err(|_| UrdfError::invalid_attribute(name, element_name(e), "invalid UTF-8"));
        }
    }
    Err(UrdfError::missing_attribute(name, element_name(e)))
}

/// Get an optional attribute value.
fn get_attribute_opt(e: &BytesStart, name: &str) -> Option<String> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == name.as_bytes() {
            return String::from_utf8(attr.value.to_vec()).ok();
        }
    }
    None
}

// ============================================================================
// MuJoCo 3.5.0's attribute reader
// ============================================================================

/// C's `isspace` in the "C" locale: ASCII whitespace and the vertical tab.
fn is_c_space(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\n' | '\u{b}' | '\u{c}' | '\r')
}

/// One number of an attribute, as MuJoCo 3.5.0's `StringToVector` reads it
/// (`user_util.cc:1238-1255`), by sim-mjcf's reader's rules (`sim-mjcf`
/// `parser/attrs.rs`, `real_token`; `numbers_read_as_sim_mjcf_reads_them`
/// holds the two to one table): a token that is not a number is a bad
/// format; a nonzero one that underflows to zero or a subnormal, or one that
/// overflows to an infinity, is MuJoCo's "number is too large"; a written
/// `NaN` or infinity, which MuJoCo loads, is refused (`D-NONFINITE`), a `NaN`
/// with a payload (`nan(1)`) included.
fn read_token(element: &str, name: &'static str, token: &str) -> Result<f64> {
    let invalid = |message: &str| UrdfError::invalid_attribute(name, element, message);
    let nan_with_payload = {
        let unsigned = token.strip_prefix(['+', '-']).unwrap_or(token);
        unsigned
            .get(..4)
            .is_some_and(|nan| nan.eq_ignore_ascii_case("nan("))
            && unsigned[4..]
                .strip_suffix(')')
                .is_some_and(|payload| !payload.contains(')'))
    };
    let Ok(x) = token.parse::<f64>() else {
        return Err(invalid(if nan_with_payload {
            "a NaN or an infinity is not allowed"
        } else {
            "bad format"
        }));
    };
    let names_infinity = token
        .trim_start_matches(['+', '-'])
        .get(..3)
        .is_some_and(|s| s.eq_ignore_ascii_case("inf"));
    let mantissa_nonzero = token
        .split(['e', 'E'])
        .next()
        .is_some_and(|m| m.bytes().any(|b| (b'1'..=b'9').contains(&b)));
    if (x.is_finite() && x.abs() < f64::MIN_POSITIVE && mantissa_nonzero)
        || (x.is_infinite() && !names_infinity)
    {
        return Err(invalid("number is too large"));
    }
    if !x.is_finite() {
        return Err(invalid("a NaN or an infinity is not allowed"));
    }
    Ok(x)
}

/// `name`'s `N` numbers as MuJoCo 3.5.0's importer reads them (`ReadAttr`,
/// `exact`, `xml_util.cc:799-813`): absent, or written with no number, it is
/// not written (`None`), refused where `required` ("required attribute
/// missing"); fewer numbers than `N` do not have enough data, more have too
/// much.
fn read_numbers<const N: usize>(
    e: &BytesStart,
    name: &'static str,
    required: bool,
) -> Result<Option<[f64; N]>> {
    let element = element_name(e);
    let values = match get_attribute_opt(e, name) {
        Some(text) => text
            .split(is_c_space)
            .filter(|t| !t.is_empty())
            .map(|t| read_token(&element, name, t))
            .collect::<Result<Vec<_>>>()?,
        None => Vec::new(),
    };
    if values.is_empty() {
        return if required {
            Err(UrdfError::missing_attribute(name, element))
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
            UrdfError::invalid_attribute(name, element, message)
        })
}

/// One number ([`read_numbers`]).
fn read_number(e: &BytesStart, name: &'static str, required: bool) -> Result<Option<f64>> {
    Ok(read_numbers::<1>(e, name, required)?.map(|[x]| x))
}

/// Three numbers ([`read_numbers`]).
fn read_vector3(
    e: &BytesStart,
    name: &'static str,
    required: bool,
) -> Result<Option<Vector3<f64>>> {
    Ok(read_numbers::<3>(e, name, required)?.map(Vector3::from))
}

/// A required number ([`read_number`]).
fn required_number(e: &BytesStart, name: &'static str) -> Result<f64> {
    read_number(e, name, true)?.ok_or_else(|| UrdfError::missing_attribute(name, element_name(e)))
}

/// A required three numbers ([`read_vector3`]).
fn required_vector3(e: &BytesStart, name: &'static str) -> Result<Vector3<f64>> {
    read_vector3(e, name, true)?.ok_or_else(|| UrdfError::missing_attribute(name, element_name(e)))
}

/// Get element name as string for error messages.
fn element_name(e: &BytesStart) -> String {
    String::from_utf8_lossy(e.name().as_ref()).to_string()
}

/// Skip an element and all its children.
fn skip_element<R: BufRead>(reader: &mut Reader<R>, name: &[u8]) -> Result<()> {
    let mut buf = Vec::new();
    let mut depth = 1;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == name => {
                depth += 1;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == name => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(())
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
        let e = BytesStart::from_content("origin xyz=\"  1   2 \t 3 \u{b} \"", 6);
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
            r#"<robot name="r"><link name="base"/><link name="arm"/><joint name="j1" type="revolute"><parent link="base"/><child link="arm"/>{origin}{extra}</joint></robot>"#
        )
    }

    /// Every document below MuJoCo 3.5.0's importer refuses (measured), for
    /// the attribute or element, or in MuJoCo's words, named; this reader
    /// refuses each. Before `a7ac2c20` 23 of the 32 loaded, 8 were refused
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
            ("mesh scale two", one_link(r#"<collision><geometry><mesh filename="a.stl" scale="1 1"/></geometry></collision>"#), "scale"),
            ("collision no geometry", one_link(r#"<collision><origin xyz="0 0 0"/></collision>"#), "geometry"),
            ("collision no shape", one_link("<collision><geometry/></collision>"), "shape"),
            ("empty collision", one_link("<collision/>"), "geometry"),
            ("visual color three", one_link(r#"<visual><geometry><sphere radius="0.1"/></geometry><material name="m"><color rgba="1 0 0"/></material></visual>"#), "rgba"),
            ("visual material unnamed", one_link(r#"<visual><geometry><sphere radius="0.1"/></geometry><material/></visual>"#), "name"),
            ("robot material color", r#"<robot name="r"><material name="m"><color rgba="1 0"/></material><link name="base"/></robot>"#.to_string(), "rgba"),
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
    /// attribute not written (the parent refused `xyz=""`, run there).
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

    #[test]
    fn test_missing_robot_name() {
        let xml = r#"<robot><link name="base"/></robot>"#;
        let result = parse_urdf_str(xml);
        assert!(result.is_err());
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
