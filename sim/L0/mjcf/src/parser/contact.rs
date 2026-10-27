//! `<contact>` pair / exclude parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfContact, MjcfContactExclude, MjcfContactPair};

use super::attrs::{Attrs, skip_element};

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
                return Err(MjcfError::eof_in("contact"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(contact)
}

/// Parse attributes of a `<contact><pair .../>` element.
pub(super) fn parse_contact_pair_attrs(e: &BytesStart) -> Result<MjcfContactPair> {
    let a = Attrs::new(e)?;
    let geom1 = a
        .string("geom1")
        .ok_or_else(|| MjcfError::missing_attribute("pair", "geom1"))?;
    let geom2 = a
        .string("geom2")
        .ok_or_else(|| MjcfError::missing_attribute("pair", "geom2"))?;

    Ok(MjcfContactPair {
        name: a.string("name"),
        class: a.string("class"),
        geom1,
        geom2,
        condim: a.int("condim")?,
        friction: a.prefix("friction")?,
        solref: a.prefix("solref")?,
        solreffriction: a.prefix("solreffriction")?,
        solimp: a.prefix("solimp")?,
        margin: a.real("margin")?,
        gap: a.real("gap")?,
    })
}

/// Parse attributes of a `<contact><exclude .../>` element.
pub(super) fn parse_contact_exclude_attrs(e: &BytesStart) -> Result<MjcfContactExclude> {
    let a = Attrs::new(e)?;
    let body1 = a
        .string("body1")
        .ok_or_else(|| MjcfError::missing_attribute("exclude", "body1"))?;
    let body2 = a
        .string("body2")
        .ok_or_else(|| MjcfError::missing_attribute("exclude", "body2"))?;

    Ok(MjcfContactExclude {
        name: a.string("name"),
        body1,
        body2,
    })
}
