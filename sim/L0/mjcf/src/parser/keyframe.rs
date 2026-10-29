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

/// Parse the `<keyframe>` element containing `<key>` children (`empty`: it
/// closed itself, so it has none).
pub(super) fn parse_keyframes<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<Vec<MjcfKeyframe>> {
    let mut keyframes = Vec::new();
    if empty {
        return Ok(keyframes);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"key" {
                    keyframes.push(parse_key_attrs(e)?);
                }
                skip_element(reader, e.name().as_ref(), empty)?;
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
