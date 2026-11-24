//! `<custom>`: its tuples, each element's object type and name, read for
//! the bodies `fusestatic` keeps and the geoms `discardvisual` keeps (MuJoCo
//! 3.5.0 resolves a tuple's elements before it fuses a body and keeps a geom
//! one names, `user_model.cc:4254`, `user_objects.cc:7918`). Its numerics and
//! texts are skipped, and no element is checked (registry
//! `D-TUPLE-UNCHECKED`).

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::types::{MjcfTuple, MjcfTupleElement};

use super::attrs::{Attrs, skip_element};

/// The tuples of a `<custom>` section (`empty`: it closed itself, so it has
/// none).
pub(super) fn parse_custom<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<Vec<MjcfTuple>> {
    let mut tuples = Vec::new();
    if empty {
        return Ok(tuples);
    }
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"tuple" {
                    tuples.push(parse_tuple(reader, e, child_empty)?);
                } else {
                    skip_element(reader, e.name().as_ref(), child_empty)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"custom" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("custom")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(tuples)
}

/// A `<tuple>`, its `name` and each `<element>`'s `objtype` and `objname`,
/// each required, as MuJoCo's reader requires them
/// (`xml_native_reader.cc:3092-3142`).
fn parse_tuple<R: BufRead>(
    reader: &mut Reader<R>,
    e: &BytesStart,
    empty: bool,
) -> Result<MjcfTuple> {
    let name = Attrs::new(e)?
        .string("name")
        .ok_or_else(|| MjcfError::missing_attribute("tuple", "name"))?;
    let mut tuple = MjcfTuple {
        name,
        elements: Vec::new(),
    };
    if empty {
        return Ok(tuple);
    }
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"element" {
                    let a = Attrs::new(e)?;
                    let objtype = a
                        .string("objtype")
                        .ok_or_else(|| MjcfError::missing_attribute("element", "objtype"))?;
                    let objname = a
                        .string("objname")
                        .ok_or_else(|| MjcfError::missing_attribute("element", "objname"))?;
                    tuple.elements.push(MjcfTupleElement { objtype, objname });
                }
                skip_element(reader, e.name().as_ref(), child_empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"tuple" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("tuple")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(tuple)
}
