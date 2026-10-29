//! `<contact>` pair / exclude parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfContact, MjcfContactExclude, MjcfContactPair};

use super::attrs::{Attrs, skip_element};

/// Parse `<contact>` element containing `<pair>` and `<exclude>` children
/// (`empty`: it closed itself, so it has none).
pub(super) fn parse_contact<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<MjcfContact> {
    let mut contact = MjcfContact::default();
    if empty {
        return Ok(contact);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                match e.name().as_ref() {
                    b"pair" => contact.pairs.push(parse_contact_pair_attrs(e)?),
                    b"exclude" => contact.excludes.push(parse_contact_exclude_attrs(e)?),
                    _ => {}
                }
                skip_element(reader, e.name().as_ref(), child_empty)?;
            }
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
