//! `<keyframe>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::MjcfKeyframe;

use super::attrs::{get_attribute_opt, parse_float_array_opt, parse_float_attr, skip_element};

// ============================================================================
// Keyframe parsing
// ============================================================================

/// Parse the `<keyframe>` element containing `<key>` children.
pub(super) fn parse_keyframes<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfKeyframe>> {
    let mut keyframes = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = e.name().as_ref().to_vec();
                match elem_name.as_slice() {
                    b"key" => {
                        keyframes.push(parse_key_attrs(e)?);
                        skip_element(reader, &elem_name)?;
                    }
                    _ => skip_element(reader, &elem_name)?,
                }
            }
            Ok(Event::Empty(ref e)) => {
                if e.name().as_ref() == b"key" {
                    keyframes.push(parse_key_attrs(e)?);
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"keyframe" => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::XmlParse("unexpected EOF in keyframe".into()));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(keyframes)
}

/// Parse attributes of a single `<key>` element.
pub(super) fn parse_key_attrs(e: &BytesStart) -> Result<MjcfKeyframe> {
    let mut kf = MjcfKeyframe::default();

    kf.name = get_attribute_opt(e, "name").unwrap_or_default();
    kf.time = parse_float_attr(e, "time").unwrap_or(0.0);
    kf.qpos = parse_float_array_opt(e, "qpos")?;
    kf.qvel = parse_float_array_opt(e, "qvel")?;
    kf.act = parse_float_array_opt(e, "act")?;
    kf.ctrl = parse_float_array_opt(e, "ctrl")?;
    kf.mpos = parse_float_array_opt(e, "mpos")?;
    kf.mquat = parse_float_array_opt(e, "mquat")?;

    Ok(kf)
}
