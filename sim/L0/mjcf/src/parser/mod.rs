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

use crate::error::{MjcfError, Result};

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
/// Returns an error if the XML is malformed or missing required elements.
pub fn parse_mjcf_str(xml: &str) -> Result<MjcfModel> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    parse_mjcf_reader(&mut reader)
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
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    model.ok_or_else(|| MjcfError::missing_element("mujoco", "MJCF document"))
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
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in mujoco".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(model)
}
