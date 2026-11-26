//! `<equality>` constraint parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    EqualityKind, MjcfConnect, MjcfDistance, MjcfEquality, MjcfFlexEquality, MjcfJointEquality,
    MjcfTendonEquality, MjcfWeld,
};

use super::attrs::{Attrs, skip_element};

/// Parse equality constraints element (`empty`: it closed itself, so it has
/// none).
pub(super) fn parse_equality<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<MjcfEquality> {
    let mut equality = MjcfEquality::new();
    if empty {
        return Ok(equality);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
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
                    name @ (b"flex" | b"flexvert") => {
                        let flex_eq = parse_flex_equality_attrs(e, name == b"flexvert")?;
                        let idx = equality.flexes.len();
                        equality.flexes.push(flex_eq);
                        equality.order.push((EqualityKind::Flex, idx));
                    }
                    _ => {}
                }
                skip_element(reader, e.name().as_ref(), child_empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"equality" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("equality")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(equality)
}

/// Every attribute MuJoCo 3.5.0's schema gives an element of `<equality>`
/// (`xml_native_reader.cc:356-368`).
const EQUALITY_ATTRIBUTES: [&str; 18] = [
    "name",
    "class",
    "body1",
    "body2",
    "anchor",
    "site1",
    "site2",
    "relpose",
    "torquescale",
    "joint1",
    "joint2",
    "polycoef",
    "tendon1",
    "tendon2",
    "flex",
    "active",
    "solref",
    "solimp",
];

/// The attributes MuJoCo 3.5.0's schema gives each element of `<equality>`.
const CONNECT_TAKES: &[&str] = &[
    "name", "class", "body1", "body2", "anchor", "site1", "site2", "active", "solref", "solimp",
];
const WELD_TAKES: &[&str] = &[
    "name",
    "class",
    "body1",
    "body2",
    "relpose",
    "anchor",
    "site1",
    "site2",
    "active",
    "solref",
    "solimp",
    "torquescale",
];
const JOINT_TAKES: &[&str] = &[
    "name", "class", "joint1", "joint2", "polycoef", "active", "solref", "solimp",
];
const TENDON_TAKES: &[&str] = &[
    "name", "class", "tendon1", "tendon2", "polycoef", "active", "solref", "solimp",
];
const FLEX_TAKES: &[&str] = &["name", "class", "flex", "active", "solref", "solimp"];

/// Refuses an attribute another element of `<equality>` takes and `element`
/// does not ([`EQUALITY_ATTRIBUTES`], `takes`), as MuJoCo 3.5.0's schema
/// refuses it before its reader reads the element ("Schema violation:
/// unrecognized attribute", measured on each element), and as this crate
/// refuses an actuator kind's (`actuator_spec.rs`). An attribute no element of
/// `<equality>` takes is not checked here.
fn refuse_foreign_attributes(a: &Attrs, element: &str, takes: &[&str]) -> Result<()> {
    match EQUALITY_ATTRIBUTES
        .iter()
        .find(|attribute| !takes.contains(attribute) && a.has(attribute))
    {
        Some(attribute) => Err(MjcfError::unknown_attribute(element, *attribute)),
        None => Ok(()),
    }
}

/// Parse connect constraint attributes.
pub(super) fn parse_connect_attrs(e: &BytesStart) -> Result<MjcfConnect> {
    let a = Attrs::new(e)?;
    refuse_foreign_attributes(&a, "connect", CONNECT_TAKES)?;
    let mut connect = MjcfConnect::default();

    connect.name = a.string("name");
    connect.class = a.string("class");

    connect.body1 = a.string("body1");
    connect.body2 = a.string("body2");
    connect.anchor = a.vec3("anchor")?;
    connect.site1 = a.string("site1");
    connect.site2 = a.string("site2");
    crate::builder::equality::connect_form(&connect)?;

    // Parse solver parameters
    connect.solimp = a.prefix("solimp")?;
    connect.solref = a.prefix("solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    connect.active = a.boolean("active")?;

    Ok(connect)
}

/// Parse weld constraint attributes.
pub(super) fn parse_weld_attrs(e: &BytesStart) -> Result<MjcfWeld> {
    let a = Attrs::new(e)?;
    refuse_foreign_attributes(&a, "weld", WELD_TAKES)?;
    let mut weld = MjcfWeld::default();

    weld.name = a.string("name");
    weld.class = a.string("class");

    weld.body1 = a.string("body1");
    weld.body2 = a.string("body2");
    weld.site1 = a.string("site1");
    weld.site2 = a.string("site2");
    weld.anchor = a.vec3("anchor")?;
    // relpose: [x y z qw qx qy qz]
    weld.relpose = a.array("relpose")?;
    crate::builder::equality::weld_form(&weld)?;
    weld.torquescale = a.real("torquescale")?;

    // Parse solver parameters
    weld.solimp = a.prefix("solimp")?;
    weld.solref = a.prefix("solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    weld.active = a.boolean("active")?;

    Ok(weld)
}

/// Parse joint equality constraint attributes.
pub(super) fn parse_joint_equality_attrs(e: &BytesStart) -> Result<MjcfJointEquality> {
    let a = Attrs::new(e)?;
    refuse_foreign_attributes(&a, "joint", JOINT_TAKES)?;
    let mut joint_eq = MjcfJointEquality::default();

    joint_eq.name = a.string("name");
    joint_eq.class = a.string("class");

    // joint1 is required
    joint_eq.joint1 = a
        .string("joint1")
        .ok_or_else(|| MjcfError::missing_attribute("joint", "joint1"))?;

    // joint2 is optional (for coupling constraints)
    joint_eq.joint2 = a.string("joint2");

    // Parse polycoef (polynomial coefficients for coupling)
    if let Some(polycoef) = polycoef::<11>(&a)? {
        joint_eq.polycoef = polycoef;
    }

    // Parse solver parameters
    joint_eq.solimp = a.prefix("solimp")?;
    joint_eq.solref = a.prefix("solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    joint_eq.active = a.boolean("active")?;

    Ok(joint_eq)
}

/// Parse distance constraint attributes.
pub(super) fn parse_distance_attrs(e: &BytesStart) -> Result<MjcfDistance> {
    let a = Attrs::new(e)?;
    let mut distance = MjcfDistance::default();

    distance.name = a.string("name");
    distance.class = a.string("class");

    // geom1 is required
    distance.geom1 = a
        .string("geom1")
        .ok_or_else(|| MjcfError::missing_attribute("distance", "geom1"))?;

    // geom2 is optional (defaults to world origin)
    distance.geom2 = a.string("geom2");

    // Parse target distance
    distance.distance = a.real("distance")?;

    // Parse solver parameters
    distance.solimp = a.prefix("solimp")?;
    distance.solref = a.prefix("solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    distance.active = a.boolean("active")?;

    Ok(distance)
}

/// Parse tendon equality constraint attributes.
pub(super) fn parse_tendon_equality_attrs(e: &BytesStart) -> Result<MjcfTendonEquality> {
    let a = Attrs::new(e)?;
    refuse_foreign_attributes(&a, "tendon", TENDON_TAKES)?;
    let mut ten_eq = MjcfTendonEquality::default();

    ten_eq.name = a.string("name");
    ten_eq.class = a.string("class");

    // tendon1 is required
    ten_eq.tendon1 = a
        .string("tendon1")
        .ok_or_else(|| MjcfError::missing_attribute("tendon", "tendon1"))?;

    // tendon2 is optional (for coupling constraints)
    ten_eq.tendon2 = a.string("tendon2");

    // Parse polycoef (polynomial coefficients for coupling)
    if let Some(polycoef) = polycoef::<5>(&a)? {
        ten_eq.polycoef = polycoef;
    }

    // Parse solver parameters
    ten_eq.solimp = a.prefix("solimp")?;
    ten_eq.solref = a.prefix("solref")?;

    // Parse active flag (Option<bool>: None = not set, cascaded from defaults)
    ten_eq.active = a.boolean("active")?;

    Ok(ten_eq)
}

/// Parse a flex equality constraint's attributes (`<equality><flex>`, or
/// `<equality><flexvert>` when `vertices`, which MuJoCo reads alike): its
/// `flex` is required (`xml_native_reader.cc:2208-2210`).
pub(super) fn parse_flex_equality_attrs(
    e: &BytesStart,
    vertices: bool,
) -> Result<MjcfFlexEquality> {
    let a = Attrs::new(e)?;
    let element = if vertices { "flexvert" } else { "flex" };
    refuse_foreign_attributes(&a, element, FLEX_TAKES)?;
    Ok(MjcfFlexEquality {
        name: a.string("name"),
        class: a.string("class"),
        flex: a
            .string("flex")
            .ok_or_else(|| MjcfError::missing_attribute(element, "flex"))?,
        solimp: a.prefix("solimp")?,
        solref: a.prefix("solref")?,
        active: a.boolean("active")?,
        vertices,
    })
}

/// `polycoef`: one up to `N` coefficients, the ones not given MuJoCo's
/// default `0 1 0 0 0` (`xml_native_reader.cc` reads five with `exact =
/// false`). A joint coupling takes up to eleven, a degree-10 polynomial: an
/// extension (D-POLYCOEF-DEGREE).
fn polycoef<const N: usize>(a: &Attrs) -> Result<Option<Vec<f64>>> {
    const DEFAULT: [f64; 5] = [0.0, 1.0, 0.0, 0.0, 0.0];
    Ok(a.prefix::<N>("polycoef")?.map(|given| {
        let mut coef = given.given().to_vec();
        if coef.len() < DEFAULT.len() {
            coef.extend_from_slice(&DEFAULT[coef.len()..]);
        }
        coef
    }))
}
