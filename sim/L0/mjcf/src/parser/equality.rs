//! `<equality>` constraint parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    EqualityKind, MjcfConnect, MjcfDistance, MjcfEquality, MjcfJointEquality, MjcfTendonEquality,
    MjcfWeld,
};

use super::attrs::{attr_fixed, get_attribute_opt, parse_float_array, parse_vector3, skip_element};

/// Parse equality constraints element.
pub(super) fn parse_equality<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfEquality> {
    let mut equality = MjcfEquality::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"connect" => {
                        let connect = parse_connect_attrs(e)?;
                        let idx = equality.connects.len();
                        equality.connects.push(connect);
                        equality.order.push((EqualityKind::Connect, idx));
                        // Skip to closing tag
                        skip_element(reader, &elem_name)?;
                    }
                    b"weld" => {
                        let weld = parse_weld_attrs(e)?;
                        let idx = equality.welds.len();
                        equality.welds.push(weld);
                        equality.order.push((EqualityKind::Weld, idx));
                        skip_element(reader, &elem_name)?;
                    }
                    b"joint" => {
                        let joint_eq = parse_joint_equality_attrs(e)?;
                        let idx = equality.joints.len();
                        equality.joints.push(joint_eq);
                        equality.order.push((EqualityKind::Joint, idx));
                        skip_element(reader, &elem_name)?;
                    }
                    b"distance" => {
                        let distance = parse_distance_attrs(e)?;
                        let idx = equality.distances.len();
                        equality.distances.push(distance);
                        equality.order.push((EqualityKind::Distance, idx));
                        skip_element(reader, &elem_name)?;
                    }
                    b"tendon" => {
                        let ten_eq = parse_tendon_equality_attrs(e)?;
                        let idx = equality.tendons.len();
                        equality.tendons.push(ten_eq);
                        equality.order.push((EqualityKind::Tendon, idx));
                        skip_element(reader, &elem_name)?;
                    }
                    // Skip other equality constraint types (flex)
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                match e.name().as_ref() {
                    b"connect" => {
                        let connect = parse_connect_attrs(e)?;
                        let idx = equality.connects.len();
                        equality.connects.push(connect);
                        equality.order.push((EqualityKind::Connect, idx));
                    }
                    b"weld" => {
                        let weld = parse_weld_attrs(e)?;
                        let idx = equality.welds.len();
                        equality.welds.push(weld);
                        equality.order.push((EqualityKind::Weld, idx));
                    }
                    b"joint" => {
                        let joint_eq = parse_joint_equality_attrs(e)?;
                        let idx = equality.joints.len();
                        equality.joints.push(joint_eq);
                        equality.order.push((EqualityKind::Joint, idx));
                    }
                    b"distance" => {
                        let distance = parse_distance_attrs(e)?;
                        let idx = equality.distances.len();
                        equality.distances.push(distance);
                        equality.order.push((EqualityKind::Distance, idx));
                    }
                    b"tendon" => {
                        let ten_eq = parse_tendon_equality_attrs(e)?;
                        let idx = equality.tendons.len();
                        equality.tendons.push(ten_eq);
                        equality.order.push((EqualityKind::Tendon, idx));
                    }
                    _ => {} // Ignore other self-closing equality constraint types
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"equality" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in equality".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(equality)
}

/// Parse connect constraint attributes.
pub(super) fn parse_connect_attrs(e: &BytesStart) -> Result<MjcfConnect> {
    let mut connect = MjcfConnect::default();

    connect.name = get_attribute_opt(e, "name");
    connect.class = get_attribute_opt(e, "class");

    // body1 is required
    connect.body1 = get_attribute_opt(e, "body1")
        .ok_or_else(|| MjcfError::missing_attribute("body1", "connect"))?;

    // body2 is optional (defaults to world)
    connect.body2 = get_attribute_opt(e, "body2");

    // Parse anchor point
    if let Some(anchor) = get_attribute_opt(e, "anchor") {
        connect.anchor = parse_vector3(&anchor)?;
    }

    // Parse solver parameters
    connect.solimp = attr_fixed::<5>(e, "solimp")?;
    connect.solref = attr_fixed::<2>(e, "solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    connect.active = get_attribute_opt(e, "active").map(|v| v == "true");

    Ok(connect)
}

/// Parse weld constraint attributes.
pub(super) fn parse_weld_attrs(e: &BytesStart) -> Result<MjcfWeld> {
    let mut weld = MjcfWeld::default();

    weld.name = get_attribute_opt(e, "name");
    weld.class = get_attribute_opt(e, "class");

    // body1 is required
    weld.body1 = get_attribute_opt(e, "body1")
        .ok_or_else(|| MjcfError::missing_attribute("body1", "weld"))?;

    // body2 is optional (defaults to world)
    weld.body2 = get_attribute_opt(e, "body2");

    // Parse anchor point
    if let Some(anchor) = get_attribute_opt(e, "anchor") {
        weld.anchor = parse_vector3(&anchor)?;
    }

    // Parse relpose [x y z qw qx qy qz]
    weld.relpose = attr_fixed::<7>(e, "relpose")?;

    // Parse solver parameters
    weld.solimp = attr_fixed::<5>(e, "solimp")?;
    weld.solref = attr_fixed::<2>(e, "solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    weld.active = get_attribute_opt(e, "active").map(|v| v == "true");

    Ok(weld)
}

/// Parse joint equality constraint attributes.
pub(super) fn parse_joint_equality_attrs(e: &BytesStart) -> Result<MjcfJointEquality> {
    let mut joint_eq = MjcfJointEquality::default();

    joint_eq.name = get_attribute_opt(e, "name");
    joint_eq.class = get_attribute_opt(e, "class");

    // joint1 is required
    joint_eq.joint1 = get_attribute_opt(e, "joint1")
        .ok_or_else(|| MjcfError::missing_attribute("joint1", "joint equality"))?;

    // joint2 is optional (for coupling constraints)
    joint_eq.joint2 = get_attribute_opt(e, "joint2");

    // Parse polycoef (polynomial coefficients for coupling)
    if let Some(polycoef) = get_attribute_opt(e, "polycoef") {
        joint_eq.polycoef = parse_float_array(&polycoef)?;
    }

    // Parse solver parameters
    joint_eq.solimp = attr_fixed::<5>(e, "solimp")?;
    joint_eq.solref = attr_fixed::<2>(e, "solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    joint_eq.active = get_attribute_opt(e, "active").map(|v| v == "true");

    Ok(joint_eq)
}

/// Parse distance constraint attributes.
pub(super) fn parse_distance_attrs(e: &BytesStart) -> Result<MjcfDistance> {
    let mut distance = MjcfDistance::default();

    distance.name = get_attribute_opt(e, "name");
    distance.class = get_attribute_opt(e, "class");

    // geom1 is required
    distance.geom1 = get_attribute_opt(e, "geom1")
        .ok_or_else(|| MjcfError::missing_attribute("geom1", "distance"))?;

    // geom2 is optional (defaults to world origin)
    distance.geom2 = get_attribute_opt(e, "geom2");

    // Parse target distance
    if let Some(dist_str) = get_attribute_opt(e, "distance") {
        distance.distance = dist_str.parse().ok();
    }

    // Parse solver parameters
    distance.solimp = attr_fixed::<5>(e, "solimp")?;
    distance.solref = attr_fixed::<2>(e, "solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    distance.active = get_attribute_opt(e, "active").map(|v| v == "true");

    Ok(distance)
}

/// Parse tendon equality constraint attributes.
pub(super) fn parse_tendon_equality_attrs(e: &BytesStart) -> Result<MjcfTendonEquality> {
    let mut ten_eq = MjcfTendonEquality::default();

    ten_eq.name = get_attribute_opt(e, "name");
    ten_eq.class = get_attribute_opt(e, "class");

    // tendon1 is required
    ten_eq.tendon1 = get_attribute_opt(e, "tendon1")
        .ok_or_else(|| MjcfError::missing_attribute("tendon1", "tendon equality"))?;

    // tendon2 is optional (for coupling constraints)
    ten_eq.tendon2 = get_attribute_opt(e, "tendon2");

    // Parse polycoef (polynomial coefficients for coupling)
    if let Some(polycoef) = get_attribute_opt(e, "polycoef") {
        ten_eq.polycoef = parse_float_array(&polycoef)?;
    }

    // Parse solver parameters
    ten_eq.solimp = attr_fixed::<5>(e, "solimp")?;
    ten_eq.solref = attr_fixed::<2>(e, "solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    ten_eq.active = get_attribute_opt(e, "active").map(|v| v == "true");

    Ok(ten_eq)
}
