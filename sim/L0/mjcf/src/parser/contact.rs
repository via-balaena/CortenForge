//! `<contact>` pair / exclude parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfContact, MjcfContactExclude, MjcfContactPair};

use super::attrs::{attr_fixed, get_attribute_opt, parse_float_attr, parse_int_attr, skip_element};

/// Parse `<contact>` element containing `<pair>` and `<exclude>` children.
pub(super) fn parse_contact<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfContact> {
    let mut contact = MjcfContact::default();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"pair" => {
                        let pair = parse_contact_pair_attrs(e)?;
                        contact.pairs.push(pair);
                        skip_element(reader, &elem_name)?;
                    }
                    b"exclude" => {
                        let exclude = parse_contact_exclude_attrs(e)?;
                        contact.excludes.push(exclude);
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"pair" => {
                    let pair = parse_contact_pair_attrs(e)?;
                    contact.pairs.push(pair);
                }
                b"exclude" => {
                    let exclude = parse_contact_exclude_attrs(e)?;
                    contact.excludes.push(exclude);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) if e.name().as_ref() == b"contact" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in contact".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(contact)
}

/// Parse attributes of a `<contact><pair .../>` element.
pub(super) fn parse_contact_pair_attrs(e: &BytesStart) -> Result<MjcfContactPair> {
    let geom1 = get_attribute_opt(e, "geom1")
        .ok_or_else(|| MjcfError::missing_attribute("geom1", "pair"))?;
    let geom2 = get_attribute_opt(e, "geom2")
        .ok_or_else(|| MjcfError::missing_attribute("geom2", "pair"))?;

    let mut pair = MjcfContactPair {
        name: get_attribute_opt(e, "name"),
        class: get_attribute_opt(e, "class"),
        geom1,
        geom2,
        condim: parse_int_attr(e, "condim"),
        friction: None,
        solref: None,
        solreffriction: None,
        solimp: None,
        margin: parse_float_attr(e, "margin"),
        gap: parse_float_attr(e, "gap"),
    };

    pair.friction = attr_fixed::<5>(e, "friction")?;
    pair.solref = attr_fixed::<2>(e, "solref")?;
    pair.solreffriction = attr_fixed::<2>(e, "solreffriction")?;
    pair.solimp = attr_fixed::<5>(e, "solimp")?;

    Ok(pair)
}

/// Parse attributes of a `<contact><exclude .../>` element.
pub(super) fn parse_contact_exclude_attrs(e: &BytesStart) -> Result<MjcfContactExclude> {
    let body1 = get_attribute_opt(e, "body1")
        .ok_or_else(|| MjcfError::missing_attribute("body1", "exclude"))?;
    let body2 = get_attribute_opt(e, "body2")
        .ok_or_else(|| MjcfError::missing_attribute("body2", "exclude"))?;

    Ok(MjcfContactExclude {
        name: get_attribute_opt(e, "name"),
        body1,
        body2,
    })
}
