//! Shared low-level XML attribute / value parsing helpers.

use nalgebra::{Vector3, Vector4};
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

/// Safe axis normalization with Z fallback for zero-length vectors.
#[inline]
pub(super) fn safe_normalize_axis(v: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { Vector3::z() }
}

/// Parse a space-separated array of unsigned integers.
pub(super) fn parse_int_array(s: &str) -> Result<Vec<u32>> {
    s.split_whitespace()
        .map(|p| {
            p.parse::<u32>()
                .map_err(|_| MjcfError::XmlParse(format!("invalid integer: {p}")))
        })
        .collect()
}

// ============================================================================
// Helper functions
// ============================================================================

/// Get an optional attribute value.
pub(super) fn get_attribute_opt(e: &BytesStart, name: &str) -> Option<String> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == name.as_bytes() {
            return String::from_utf8(attr.value.to_vec()).ok();
        }
    }
    None
}

/// Parse a float attribute, returning None if not present or invalid.
pub(super) fn parse_float_attr(e: &BytesStart, name: &str) -> Option<f64> {
    get_attribute_opt(e, name).and_then(|s| s.parse().ok())
}

/// Parse an integer attribute.
pub(super) fn parse_int_attr(e: &BytesStart, name: &str) -> Option<i32> {
    get_attribute_opt(e, name).and_then(|s| s.parse().ok())
}

/// Parse `maxhullvert` value: -1 → None (no limit), >= 4 → Some(n), else error.
/// MuJoCo ref: `xml_native_reader.cc` validates `n != -1 && n < 4` → error.
pub(super) fn parse_maxhullvert(n: i32) -> Result<Option<usize>> {
    if n == -1 {
        Ok(None)
    } else if n >= 4 {
        // `n >= 4` from the branch guard.
        #[allow(clippy::cast_sign_loss)]
        Ok(Some(n as usize))
    } else {
        Err(MjcfError::XmlParse(
            "maxhullvert must be larger than 3".to_string(),
        ))
    }
}

/// Parse a space-separated vector3 string.
pub(super) fn parse_vector3(s: &str) -> Result<Vector3<f64>> {
    let parts = parse_float_array(s)?;

    if parts.len() < 3 {
        return Err(MjcfError::XmlParse(format!(
            "expected 3 values in vector, got {}: {s}",
            parts.len()
        )));
    }

    Ok(Vector3::new(parts[0], parts[1], parts[2]))
}

/// Parse a space-separated vector4 string.
pub(super) fn parse_vector4(s: &str) -> Result<Vector4<f64>> {
    let parts = parse_float_array(s)?;

    if parts.len() < 4 {
        return Err(MjcfError::XmlParse(format!(
            "expected 4 values in vector, got {}: {s}",
            parts.len()
        )));
    }

    Ok(Vector4::new(parts[0], parts[1], parts[2], parts[3]))
}

/// Parse a space-separated array of floats.
pub(super) fn parse_float_array(s: &str) -> Result<Vec<f64>> {
    s.split_whitespace()
        .map(|p| {
            p.parse::<f64>()
                .map_err(|_| MjcfError::XmlParse(format!("invalid float: {p}")))
        })
        .collect()
}

/// Parse an optional space-separated float array attribute.
/// Returns `Ok(None)` when absent, `Ok(Some(vec))` when present and valid,
/// `Err` when present but contains unparseable floats.
pub(super) fn parse_float_array_opt(e: &BytesStart, name: &str) -> Result<Option<Vec<f64>>> {
    match get_attribute_opt(e, name) {
        Some(s) => Ok(Some(parse_float_array(&s)?)),
        None => Ok(None),
    }
}

/// Optional `vector3` attribute: `Ok(None)` when absent, else parsed via
/// [`parse_vector3`] (errors on fewer than 3 values, takes the first 3).
pub(super) fn attr_vec3(e: &BytesStart, name: &str) -> Result<Option<Vector3<f64>>> {
    match get_attribute_opt(e, name) {
        Some(s) => Ok(Some(parse_vector3(&s)?)),
        None => Ok(None),
    }
}

/// Optional `vector4` attribute: `Ok(None)` when absent, else parsed via
/// [`parse_vector4`] (errors on fewer than 4 values, takes the first 4).
pub(super) fn attr_vec4(e: &BytesStart, name: &str) -> Result<Option<Vector4<f64>>> {
    match get_attribute_opt(e, name) {
        Some(s) => Ok(Some(parse_vector4(&s)?)),
        None => Ok(None),
    }
}

/// Optional boolean attribute. `Some(true)` only when the value is exactly
/// `"true"` (matching MuJoCo's reader); `Some(false)` for any other value;
/// `None` when absent.
pub(super) fn attr_bool(e: &BytesStart, name: &str) -> Option<bool> {
    get_attribute_opt(e, name).map(|s| s == "true")
}

/// Optional fixed-length float-array attribute.
///
/// Preserves the historical lenient behavior of the hand-unrolled call sites:
/// `None` when the attribute is absent **or** carries fewer than `N` values;
/// otherwise the first `N` values. (Tightening this to reject a wrong arity —
/// matching MuJoCo and the existing strict `!= N` sites like `fluidcoef` — is a
/// deliberate, conformance-gated follow-up; centralizing it here makes that a
/// one-line change.)
pub(super) fn attr_fixed<const N: usize>(e: &BytesStart, name: &str) -> Result<Option<[f64; N]>> {
    match get_attribute_opt(e, name) {
        Some(s) => {
            let parts = parse_float_array(&s)?;
            if parts.len() >= N {
                let mut arr = [0.0; N];
                arr.copy_from_slice(&parts[..N]);
                Ok(Some(arr))
            } else {
                Ok(None)
            }
        }
        None => Ok(None),
    }
}

/// Skip an element and all its children.
pub(super) fn skip_element<R: BufRead>(reader: &mut Reader<R>, name: &[u8]) -> Result<()> {
    let mut buf = Vec::new();
    let mut depth = 1;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == name => {
                depth += 1;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == name => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(())
}
