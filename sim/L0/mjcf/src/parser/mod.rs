//! MJCF XML parser.
//!
//! Parses MJCF XML into the intermediate representation types.

mod actuator;
mod asset;
mod attrs;
mod body;
mod composite;
mod contact;
mod defaults;
mod deformable;
mod equality;
mod extension;
mod keyframe;
mod options;
mod sensor;
mod tendon;

#[cfg(test)]
mod tests;

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{Location, MjcfError, Result};

use crate::types::{EqualityKind, MjcfContact, MjcfModel};

use self::actuator::parse_actuators;
use self::asset::parse_asset;
use self::attrs::{get_attribute_opt, skip_element};
use self::body::parse_worldbody;
use self::contact::parse_contact;
use self::defaults::parse_default;
use self::deformable::parse_deformable;
use self::equality::parse_equality;
use self::extension::parse_extension;
use self::keyframe::parse_keyframes;
use self::options::{parse_compiler_attrs, parse_option, parse_option_attrs, parse_size_attrs};
use self::sensor::parse_sensors;
use self::tendon::parse_tendons;

/// Parse an MJCF string into a model.
///
/// # Errors
///
/// Returns an error if the XML is malformed, nested deeper than MuJoCo's XML
/// reader takes, or missing required elements.
pub fn parse_mjcf_str(xml: &str) -> Result<MjcfModel> {
    crate::stack::on_large_stack(|| parse_mjcf_unwrapped(xml))
}

/// MuJoCo 3.5.0's XML reader (tinyxml2) takes an element that is not
/// self-closing, an empty `<body></body>` included, at depth 498 at most, the
/// root at depth 1 (measured, research A4 §1.1); a self-closing one may sit one
/// deeper. It bounds the parser's recursion.
const XML_DEPTH_LIMIT: usize = 498;

/// [`parse_mjcf_str`]'s work, on the caller's stack.
pub fn parse_mjcf_unwrapped(xml: &str) -> Result<MjcfModel> {
    check_xml_depth(xml)?;
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    parse_mjcf_reader(&mut reader).map_err(|e| {
        // A syntax error stops the reader where it was found, so it is named
        // there; any other error was raised after the reader read the element
        // it is about.
        let at = match reader.error_position() {
            0 => locate(xml, offset_of(reader.buffer_position(), xml)),
            p => locate_syntax_error(xml, p),
        };
        e.with_location_if_unknown(at)
    })
}

/// MuJoCo's XML reader's depth rule, read without recursing. An error the
/// XML reader raises ends the check, for the parser to report; a malformed
/// attribute is not one, so a document with one is refused as too deep when
/// it nests past the limit, where MuJoCo refuses it for the attribute.
fn check_xml_depth(xml: &str) -> Result<()> {
    let mut reader = Reader::from_str(xml);
    let mut depth = 0usize;
    loop {
        let event = reader.read_event();
        let after = usize::try_from(reader.buffer_position()).unwrap_or(xml.len());
        match event {
            Ok(Event::Start(_)) => {
                depth += 1;
                if depth > XML_DEPTH_LIMIT {
                    return Err(MjcfError::too_deep(XML_DEPTH_LIMIT).at(locate(xml, after)));
                }
            }
            Ok(Event::End(_)) => depth = depth.saturating_sub(1),
            Ok(Event::Eof) | Err(_) => return Ok(()),
            Ok(_) => {}
        }
    }
}

/// Where byte `offset` of `xml` falls: the line of the last element that
/// starts before it, and the path of elements open at that element (the root
/// `<mujoco>` left out, the last eight kept).
fn locate(xml: &str, offset: usize) -> Location {
    let (end, _, last) = scan_to(xml, offset);
    let Some((before, path)) = last else {
        return Location::parsed(line_at(xml, end), String::new());
    };
    let tag = xml[before..end].find('<').map_or(before, |i| before + i);
    Location::parsed(line_at(xml, tag), show_path(&path))
}

/// Where a syntax error the XML reader raised at byte `position` of `xml` is:
/// that byte's line, and the path of elements open there.
pub fn locate_syntax_error(xml: &str, position: u64) -> Location {
    let (end, open, _) = scan_to(xml, offset_of(position, xml));
    Location::parsed(line_at(xml, end), show_path(&open))
}

/// A reader's byte position as an offset into `xml`.
fn offset_of(position: u64, xml: &str) -> usize {
    usize::try_from(position).unwrap_or(xml.len())
}

/// `xml` read up to byte `offset` (moved back to a character boundary): that
/// end, the elements open there, and the last element begun before it with
/// its start and the path of elements to it.
fn scan_to(xml: &str, offset: usize) -> (usize, Vec<String>, Option<(usize, Vec<String>)>) {
    let mut end = offset.min(xml.len());
    while !xml.is_char_boundary(end) {
        end -= 1;
    }
    let mut reader = Reader::from_str(&xml[..end]);
    let mut open: Vec<String> = Vec::new();
    let mut last: Option<(usize, Vec<String>)> = None;
    loop {
        let before = usize::try_from(reader.buffer_position()).unwrap_or(end);
        match reader.read_event() {
            Ok(Event::Start(e)) => {
                open.push(segment(&e));
                last = Some((before, open.clone()));
            }
            Ok(Event::Empty(e)) => {
                let mut path = open.clone();
                path.push(segment(&e));
                last = Some((before, path));
            }
            Ok(Event::End(_)) => {
                open.pop();
            }
            Ok(Event::Eof) | Err(_) => break,
            Ok(_) => {}
        }
    }
    (end, open, last)
}

/// A path of elements, the root `<mujoco>` left out and the last eight kept.
fn show_path(path: &[String]) -> String {
    let path: Vec<&str> = path
        .iter()
        .map(String::as_str)
        .skip_while(|s| *s == "mujoco")
        .collect();
    if path.len() > 8 {
        format!("… > {}", path[path.len() - 8..].join(" > "))
    } else {
        path.join(" > ")
    }
}

/// An element's tag, with its name when it has one (`body 'a'`).
fn segment(e: &BytesStart) -> String {
    let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
    match get_attribute_opt(e, "name") {
        Some(name) => format!("{tag} '{name}'"),
        None => tag,
    }
}

/// The 1-based line of byte `offset`.
fn line_at(xml: &str, offset: usize) -> usize {
    xml[..offset].matches('\n').count() + 1
}

/// Parse MJCF from a reader.
fn parse_mjcf_reader<R: BufRead>(reader: &mut Reader<R>) -> Result<MjcfModel> {
    let mut buf = Vec::new();
    let mut model: Option<MjcfModel> = None;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"mujoco" => {
                model = Some(parse_mujoco(reader, e)?);
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    model.ok_or_else(|| MjcfError::xml("no <mujoco> root element"))
}

/// Parse the mujoco root element and its children.
fn parse_mujoco<R: BufRead>(reader: &mut Reader<R>, start: &BytesStart) -> Result<MjcfModel> {
    let name = get_attribute_opt(start, "model").unwrap_or_else(|| "unnamed".to_string());
    let mut model = MjcfModel::new(name);
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"option" => {
                        model.option = parse_option(reader, e)?;
                    }
                    b"compiler" => {
                        model.compiler = parse_compiler_attrs(e)?;
                        skip_element(reader, &elem_name)?;
                    }
                    b"default" => {
                        let defaults = parse_default(reader, e, None)?;
                        model.defaults.extend(defaults);
                    }
                    b"asset" => {
                        let (meshes, hfields) = parse_asset(reader)?;
                        model.meshes.extend(meshes);
                        model.hfields.extend(hfields);
                    }
                    b"worldbody" => {
                        let wb = parse_worldbody(reader)?;
                        // Merge: append children, geoms, sites, frames, composites into existing worldbody
                        model.worldbody.children.extend(wb.children);
                        model.worldbody.geoms.extend(wb.geoms);
                        model.worldbody.sites.extend(wb.sites);
                        model.worldbody.joints.extend(wb.joints);
                        model.worldbody.frames.extend(wb.frames);
                        model.worldbody.composites.extend(wb.composites);
                    }
                    b"actuator" => {
                        let actuators = parse_actuators(reader)?;
                        model.actuators.extend(actuators);
                    }
                    b"equality" => {
                        let eq = parse_equality(reader)?;
                        // Offset order indices before merging (indices are relative
                        // to the parsed block's vecs, but must reference positions
                        // in the merged vecs).
                        let co = model.equality.connects.len();
                        let wo = model.equality.welds.len();
                        let jo = model.equality.joints.len();
                        let do_ = model.equality.distances.len();
                        let to = model.equality.tendons.len();
                        for &(kind, idx) in &eq.order {
                            let offset_idx = match kind {
                                EqualityKind::Connect => idx + co,
                                EqualityKind::Weld => idx + wo,
                                EqualityKind::Joint => idx + jo,
                                EqualityKind::Distance => idx + do_,
                                EqualityKind::Tendon => idx + to,
                            };
                            model.equality.order.push((kind, offset_idx));
                        }
                        model.equality.connects.extend(eq.connects);
                        model.equality.welds.extend(eq.welds);
                        model.equality.joints.extend(eq.joints);
                        model.equality.distances.extend(eq.distances);
                        model.equality.tendons.extend(eq.tendons);
                    }
                    b"deformable" => {
                        let (skins, flex) = parse_deformable(reader)?;
                        model.skins.extend(skins);
                        model.flex.extend(flex);
                    }
                    b"tendon" => {
                        let tendons = parse_tendons(reader)?;
                        model.tendons.extend(tendons);
                    }
                    b"sensor" => {
                        let sensors = parse_sensors(reader)?;
                        model.sensors.extend(sensors);
                    }
                    b"contact" => {
                        let ct = parse_contact(reader)?;
                        model.contact.pairs.extend(ct.pairs);
                        model.contact.excludes.extend(ct.excludes);
                    }
                    b"keyframe" => {
                        let kfs = parse_keyframes(reader)?;
                        model.keyframes.extend(kfs);
                    }
                    b"size" => {
                        parse_size_attrs(e, &mut model);
                        skip_element(reader, &elem_name)?;
                    }
                    b"extension" => {
                        let ext = parse_extension(reader)?;
                        model.extensions.push(ext);
                    }
                    // Skip other elements
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                // Handle self-closing elements
                if e.name().as_ref() == b"option" {
                    model.option = parse_option_attrs(e)?;
                } else if e.name().as_ref() == b"compiler" {
                    model.compiler = parse_compiler_attrs(e)?;
                } else if e.name().as_ref() == b"contact" {
                    model.contact = MjcfContact::default();
                } else if e.name().as_ref() == b"keyframe" {
                    // Empty <keyframe/> — no keyframes defined. model.keyframes is
                    // already Vec::new() from Default, so nothing to do.
                } else if e.name().as_ref() == b"size" {
                    parse_size_attrs(e, &mut model);
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"mujoco" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("mujoco")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(model)
}
