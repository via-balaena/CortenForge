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
                    // Skip material, gazebo, and other elements
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
                        if let Ok(visual) = parse_visual(reader, e) {
                            link.visuals.push(visual);
                        }
                    }
                    b"collision" => {
                        if let Ok(collision) = parse_collision(reader, e) {
                            link.collisions.push(collision);
                        }
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"link" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in link".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(link)
}

/// Parse an inertial element.
fn parse_inertial<R: BufRead>(reader: &mut Reader<R>) -> Result<UrdfInertial> {
    let mut inertial = UrdfInertial::default();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                match e.name().as_ref() {
                    b"origin" => {
                        inertial.origin = parse_origin(e)?;
                    }
                    b"mass" => {
                        inertial.mass = parse_mass(e)?;
                    }
                    b"inertia" => {
                        inertial.inertia = parse_inertia_element(e)?;
                    }
                    _ => {
                        // For Start events in unknown elements, skip them
                    }
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"inertial" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in inertial".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(inertial)
}

/// Parse origin element attributes.
fn parse_origin(e: &BytesStart) -> Result<UrdfOrigin> {
    let xyz = get_attribute_opt(e, "xyz")
        .map(|s| parse_vector3(&s))
        .transpose()?
        .unwrap_or_else(Vector3::zeros);

    let rpy = get_attribute_opt(e, "rpy")
        .map(|s| parse_vector3(&s))
        .transpose()?
        .unwrap_or_else(Vector3::zeros);

    Ok(UrdfOrigin::new(xyz, rpy))
}

/// Parse mass element.
fn parse_mass(e: &BytesStart) -> Result<f64> {
    let value_str = get_attribute(e, "value")?;
    value_str
        .parse()
        .map_err(|_| UrdfError::invalid_attribute("value", "mass", "expected a number"))
}

/// Parse inertia element attributes.
fn parse_inertia_element(e: &BytesStart) -> Result<UrdfInertia> {
    Ok(UrdfInertia {
        ixx: parse_float_attr(e, "ixx").unwrap_or(0.0),
        ixy: parse_float_attr(e, "ixy").unwrap_or(0.0),
        ixz: parse_float_attr(e, "ixz").unwrap_or(0.0),
        iyy: parse_float_attr(e, "iyy").unwrap_or(0.0),
        iyz: parse_float_attr(e, "iyz").unwrap_or(0.0),
        izz: parse_float_attr(e, "izz").unwrap_or(0.0),
    })
}

/// Parse a visual element.
fn parse_visual<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<UrdfVisual> {
    let name = get_attribute_opt(start, "name");
    let mut origin = UrdfOrigin::default();
    let mut geometry: Option<UrdfGeometry> = None;
    let mut material: Option<String> = None;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"origin" => origin = parse_origin(e)?,
                    b"geometry" => {
                        geometry = Some(parse_geometry(reader)?);
                    }
                    b"material" => {
                        material = get_attribute_opt(e, "name");
                        // Skip material content if it's a start tag
                        if matches!(buf.first(), Some(_)) {
                            // It was parsed as Start, skip to end
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"visual" => break,
            Ok(Event::Eof) => return Err(UrdfError::XmlParse("unexpected EOF in visual".into())),
            Ok(_) => {}
            Err(e) => return Err(UrdfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    let geometry = geometry.ok_or_else(|| UrdfError::missing_element("geometry", "visual"))?;

    Ok(UrdfVisual {
        name,
        origin,
        geometry,
        material,
    })
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
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"origin" => origin = parse_origin(e)?,
                b"geometry" => {
                    geometry = Some(parse_geometry(reader)?);
                }
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
                b"box" => {
                    let size = get_attribute(e, "size")?;
                    let size = parse_vector3(&size)?;
                    geometry = Some(UrdfGeometry::Box { size });
                }
                b"cylinder" => {
                    let radius = parse_float_attr(e, "radius")
                        .ok_or_else(|| UrdfError::missing_attribute("radius", "cylinder"))?;
                    let length = parse_float_attr(e, "length")
                        .ok_or_else(|| UrdfError::missing_attribute("length", "cylinder"))?;
                    geometry = Some(UrdfGeometry::Cylinder { radius, length });
                }
                b"sphere" => {
                    let radius = parse_float_attr(e, "radius")
                        .ok_or_else(|| UrdfError::missing_attribute("radius", "sphere"))?;
                    geometry = Some(UrdfGeometry::Sphere { radius });
                }
                b"mesh" => {
                    let filename = get_attribute(e, "filename")?;
                    let scale = get_attribute_opt(e, "scale")
                        .map(|s| parse_vector3(&s))
                        .transpose()?;
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
    let mut axis = Vector3::z();
    let mut limit: Option<UrdfJointLimit> = None;
    let mut dynamics: Option<UrdfJointDynamics> = None;

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
                    b"axis" => {
                        if let Some(xyz) = get_attribute_opt(e, "xyz") {
                            axis = parse_vector3(&xyz)?;
                        }
                    }
                    b"limit" => {
                        limit = Some(parse_joint_limit(e)?);
                    }
                    b"dynamics" => {
                        dynamics = Some(parse_joint_dynamics(e)?);
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

    Ok(joint)
}

/// Parse joint limit element.
fn parse_joint_limit(e: &BytesStart) -> Result<UrdfJointLimit> {
    Ok(UrdfJointLimit {
        lower: parse_float_attr(e, "lower").unwrap_or(0.0),
        upper: parse_float_attr(e, "upper").unwrap_or(0.0),
        effort: parse_float_attr(e, "effort").unwrap_or(0.0),
        velocity: parse_float_attr(e, "velocity").unwrap_or(0.0),
    })
}

/// Parse joint dynamics element.
fn parse_joint_dynamics(e: &BytesStart) -> Result<UrdfJointDynamics> {
    Ok(UrdfJointDynamics {
        damping: parse_float_attr(e, "damping").unwrap_or(0.0),
        friction: parse_float_attr(e, "friction").unwrap_or(0.0),
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

/// Parse a float attribute, returning None if not present or invalid.
fn parse_float_attr(e: &BytesStart, name: &str) -> Option<f64> {
    get_attribute_opt(e, name).and_then(|s| s.parse().ok())
}

/// Parse a space-separated vector3 string.
fn parse_vector3(s: &str) -> Result<Vector3<f64>> {
    let parts: Vec<f64> = s
        .split_whitespace()
        .map(|p| p.parse::<f64>())
        .collect::<std::result::Result<Vec<_>, _>>()
        .map_err(|_| UrdfError::XmlParse(format!("invalid vector3: {s}")))?;

    if parts.len() != 3 {
        return Err(UrdfError::XmlParse(format!(
            "expected 3 values in vector, got {}: {s}",
            parts.len()
        )));
    }

    Ok(Vector3::new(parts[0], parts[1], parts[2]))
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
#[allow(clippy::unwrap_used, clippy::expect_used)]
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
                        <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
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
        assert_relative_eq!(limit.lower, -1.57, epsilon = 1e-10);
        assert_relative_eq!(limit.upper, 1.57, epsilon = 1e-10);
    }

    #[test]
    fn test_parse_origin() {
        let xml = r#"
            <robot name="test">
                <link name="base">
                    <inertial>
                        <origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
                        <mass value="1"/>
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
    fn test_parse_vector3() {
        let v = parse_vector3("1.0 2.0 3.0").expect("should parse");
        assert_relative_eq!(v.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(v.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(v.z, 3.0, epsilon = 1e-10);

        // With extra whitespace
        let v = parse_vector3("  1   2   3  ").expect("should parse");
        assert_relative_eq!(v.x, 1.0, epsilon = 1e-10);
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
