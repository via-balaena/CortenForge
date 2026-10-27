//! `<keyframe>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::MjcfKeyframe;

use super::attrs::{Attrs, skip_element};

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
                return Err(MjcfError::eof_in("keyframe"));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(keyframes)
}

/// Parse attributes of a single `<key>` element.
pub(super) fn parse_key_attrs(e: &BytesStart) -> Result<MjcfKeyframe> {
    let a = Attrs::new(e)?;
    let mut kf = MjcfKeyframe::default();

    kf.name = a.string("name").unwrap_or_default();
    kf.time = a.real("time")?.unwrap_or(0.0);
    kf.qpos = a.vector("qpos")?;
    kf.qvel = a.vector("qvel")?;
    kf.act = a.vector("act")?;
    kf.ctrl = a.vector("ctrl")?;
    kf.mpos = a.vector("mpos")?;
    kf.mquat = a.vector("mquat")?;

    Ok(kf)
}
