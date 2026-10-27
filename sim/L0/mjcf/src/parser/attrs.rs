//! The attribute reader, and the parser's other low-level helpers.
//!
//! [`Attrs`] reads an element's attributes with MuJoCo 3.5.0's rules
//! (`xml_util.cc` `ReadAttr`, `ReadAttrInt`; `user_util.cc` `StringToVector`):
//! - a number list is split on C's `isspace` (a vertical tab included) and
//!   each token read whole: a token that is not a number is refused, and so is
//!   one whose nonzero digits underflow to zero or a subnormal (MuJoCo's
//!   "number is too large"; `1e-999` and `1e-310` measured);
//! - a written NaN or infinity is refused, where MuJoCo loads it
//!   (D-NONFINITE), and a number that overflows is out of range, as MuJoCo's
//!   "number is too large";
//! - an empty or all-space number list is the attribute absent, except for an
//!   integer read alone, which is refused (measured);
//! - an attribute takes exactly its count, or, where MuJoCo reads it with
//!   `exact = false`, one up to its count ([`Prefix`]); more is always refused;
//! - text is read as tinyxml2 gives it to MuJoCo ([`unescape`]); a malformed
//!   attribute, or one given twice, is an XML error.
//!
//! Hexadecimal floats (`0x1p1`) are refused, where MuJoCo's `strtod` reads
//! them (D-HEX-FLOAT).

use nalgebra::{Vector3, Vector4};
use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;
use std::num::IntErrorKind;

use crate::error::{MjcfError, Result, ValueCount};
use crate::types::Prefix;

/// Safe axis normalization with Z fallback for zero-length vectors.
#[inline]
pub(super) fn safe_normalize_axis(v: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { Vector3::z() }
}

/// C's `isspace` in the "C" locale: Rust's ASCII whitespace and the vertical tab.
fn is_c_space(c: char) -> bool {
    matches!(c, ' ' | '\t' | '\n' | '\u{b}' | '\u{c}' | '\r')
}

/// Whether a token spells an infinity (`inf`, `-Infinity`), as Rust's parser
/// reads one, rather than overflowing to it.
fn names_infinity(token: &str) -> bool {
    token
        .trim_start_matches(['+', '-'])
        .get(..3)
        .is_some_and(|s| s.eq_ignore_ascii_case("inf"))
}

/// Whether a decimal number's mantissa has a nonzero digit, so that reading it
/// as zero or a subnormal is an underflow.
fn mantissa_is_nonzero(token: &str) -> bool {
    token
        .split(['e', 'E'])
        .next()
        .is_some_and(|m| m.bytes().any(|b| (b'1'..=b'9').contains(&b)))
}

/// An attribute's text as tinyxml2 gives it to MuJoCo 3.5.0 (`StrPair::GetStr`):
/// line ends made `\n`; the five predefined entities and numeric character
/// references replaced; any other `&` kept as written (measured: `a&amp;b` is
/// `a&b`, while `a & b` and `a&foo;b` load as written).
fn unescape(raw: &str) -> String {
    const ENTITIES: [(&str, char); 5] = [
        ("quot;", '"'),
        ("amp;", '&'),
        ("apos;", '\''),
        ("lt;", '<'),
        ("gt;", '>'),
    ];
    let mut out = String::with_capacity(raw.len());
    let mut rest = raw;
    while let Some(c) = rest.chars().next() {
        let after = &rest[c.len_utf8()..];
        match c {
            '\r' | '\n' => {
                out.push('\n');
                let other = if c == '\r' { '\n' } else { '\r' };
                rest = after.strip_prefix(other).unwrap_or(after);
            }
            '&' => {
                if let Some((ch, len)) = character_reference(after) {
                    out.push(ch);
                    rest = &after[len..];
                } else if let Some((pattern, ch)) = ENTITIES
                    .iter()
                    .find(|(pattern, _)| after.starts_with(pattern))
                {
                    out.push(*ch);
                    rest = &after[pattern.len()..];
                } else {
                    out.push('&');
                    rest = after;
                }
            }
            _ => {
                out.push(c);
                rest = after;
            }
        }
    }
    out
}

/// A numeric character reference after its `&` (`#20013;`, `#x4e2d;`): the
/// character and the length read.
fn character_reference(after_amp: &str) -> Option<(char, usize)> {
    let body = after_amp.strip_prefix('#')?;
    let (digits, radix, prefix) = match body.strip_prefix('x') {
        Some(hex) => (hex, 16, 2),
        None => (body, 10, 1),
    };
    let end = digits.find(';')?;
    let code = u32::from_str_radix(&digits[..end], radix).ok()?;
    Some((char::from_u32(code)?, prefix + end + 1))
}

/// The attributes of one element, each read once, as tinyxml2 gives them.
pub(super) struct Attrs {
    element: String,
    list: Vec<(String, String)>,
}

impl Attrs {
    /// Reads every attribute of `e`. A malformed attribute (`size=`) or one
    /// given twice is an XML error, as tinyxml2's `XML_ERROR_PARSING_ATTRIBUTE`
    /// is in MuJoCo (measured).
    pub(super) fn new(e: &BytesStart) -> Result<Self> {
        let element = String::from_utf8_lossy(e.name().as_ref()).into_owned();
        let mut list = Vec::new();
        for attr in e.attributes() {
            let attr = attr.map_err(MjcfError::xml)?;
            let raw = std::str::from_utf8(&attr.value).map_err(MjcfError::xml)?;
            list.push((
                String::from_utf8_lossy(attr.key.as_ref()).into_owned(),
                unescape(raw),
            ));
        }
        Ok(Self { element, list })
    }

    /// The element's tag.
    pub(super) fn element(&self) -> &str {
        &self.element
    }

    /// The attribute's text, as written but for its entities.
    pub(super) fn text(&self, name: &str) -> Option<&str> {
        self.list
            .iter()
            .find(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
    }

    /// The attribute's text, owned.
    pub(super) fn string(&self, name: &str) -> Option<String> {
        self.text(name).map(str::to_owned)
    }

    /// Whether the attribute is written.
    pub(super) fn has(&self, name: &str) -> bool {
        self.text(name).is_some()
    }

    /// `Some(true)` for exactly `true`, `Some(false)` for any other text.
    /// MuJoCo refuses any word but `false` and `true`.
    pub(super) fn is_true(&self, name: &str) -> Option<bool> {
        self.text(name).map(|s| s == "true")
    }

    /// The attribute's numbers, any count; `None` when absent or empty.
    pub(super) fn vector(&self, name: &str) -> Result<Option<Vec<f64>>> {
        let Some(text) = self.text(name) else {
            return Ok(None);
        };
        let values = text
            .split(is_c_space)
            .filter(|t| !t.is_empty())
            .map(|t| self.real_token(name, t))
            .collect::<Result<Vec<_>>>()?;
        Ok((!values.is_empty()).then_some(values))
    }

    /// Exactly `N` numbers.
    pub(super) fn array<const N: usize>(&self, name: &str) -> Result<Option<[f64; N]>> {
        let Some(values) = self.vector(name)? else {
            return Ok(None);
        };
        <[f64; N]>::try_from(values.as_slice())
            .map(Some)
            .map_err(|_| self.wrong_count(name, ValueCount::Exactly(N), values.len()))
    }

    /// One number.
    pub(super) fn real(&self, name: &str) -> Result<Option<f64>> {
        Ok(self.array::<1>(name)?.map(|[x]| x))
    }

    /// Exactly three numbers.
    pub(super) fn vec3(&self, name: &str) -> Result<Option<Vector3<f64>>> {
        Ok(self.array::<3>(name)?.map(Vector3::from))
    }

    /// Exactly four numbers.
    pub(super) fn vec4(&self, name: &str) -> Result<Option<Vector4<f64>>> {
        Ok(self.array::<4>(name)?.map(Vector4::from))
    }

    /// A quaternion, as MuJoCo's `ReadQuat` reads one (`xml_util.cc:830-843`):
    /// exactly four numbers, not all zero (measured: "zero quaternion is not
    /// allowed").
    pub(super) fn quat(&self, name: &str) -> Result<Option<Vector4<f64>>> {
        let Some(q) = self.vec4(name)? else {
            return Ok(None);
        };
        if q.iter().all(|&x| x == 0.0) {
            return Err(MjcfError::invalid_value(
                &self.element,
                name,
                "zero quaternion is not allowed",
            ));
        }
        Ok(Some(q))
    }

    /// At most one orientation, as MuJoCo's `ReadAlternative` counts them
    /// (`xml_base.cc:54-77`): `quat` when it is written, even empty, and
    /// `axisangle`, `xyaxes`, `zaxis` and `euler` when they hold a value
    /// (measured: `quat="" euler="0 0 30"` is refused, `quat="1 0 0 0"
    /// euler=""` loads).
    pub(super) fn one_orientation(&self) -> Result<()> {
        let mut given = Vec::new();
        if self.has("quat") {
            given.push("quat");
        }
        for name in ["axisangle", "xyaxes", "zaxis", "euler"] {
            if self
                .text(name)
                .is_some_and(|t| t.split(is_c_space).any(|s| !s.is_empty()))
            {
                given.push(name);
            }
        }
        if given.len() > 1 {
            return Err(MjcfError::conflicting(&self.element, &given));
        }
        Ok(())
    }

    /// One up to `N` numbers (MuJoCo's `exact = false`).
    pub(super) fn prefix<const N: usize>(&self, name: &str) -> Result<Option<Prefix<N>>> {
        let Some(values) = self.vector(name)? else {
            return Ok(None);
        };
        Prefix::new(&values)
            .map(Some)
            .ok_or_else(|| self.wrong_count(name, ValueCount::AtMost(N), values.len()))
    }

    /// One integer, as MuJoCo's `ReadAttrInt` reads it: spaces around it and a
    /// sign are taken; empty text, a fraction, an exponent, a second number or
    /// a value outside `i32` are refused (measured).
    pub(super) fn int(&self, name: &str) -> Result<Option<i32>> {
        let Some(text) = self.text(name) else {
            return Ok(None);
        };
        let tokens: Vec<&str> = text.split(is_c_space).filter(|t| !t.is_empty()).collect();
        match tokens.as_slice() {
            [] => Err(self.bad_number(name, text)),
            [token] => token
                .parse::<i32>()
                .map(Some)
                .map_err(|_| self.bad_number(name, token)),
            _ => Err(self.wrong_count(name, ValueCount::Exactly(1), tokens.len())),
        }
    }

    /// Integers, any count, as MuJoCo's `ReadAttr` reads them (`strtol`): `None`
    /// when absent or empty; a value outside `i32` is out of range.
    pub(super) fn int_vector(&self, name: &str) -> Result<Option<Vec<i32>>> {
        let Some(text) = self.text(name) else {
            return Ok(None);
        };
        let values = text
            .split(is_c_space)
            .filter(|t| !t.is_empty())
            .map(|t| {
                t.parse::<i32>().map_err(|e| match e.kind() {
                    IntErrorKind::PosOverflow | IntErrorKind::NegOverflow => {
                        MjcfError::out_of_range_in(&self.element, name, t)
                    }
                    _ => self.bad_number(name, t),
                })
            })
            .collect::<Result<Vec<_>>>()?;
        Ok((!values.is_empty()).then_some(values))
    }

    /// Exactly `N` integers, read as [`Attrs::int_vector`].
    pub(super) fn int_array<const N: usize>(&self, name: &str) -> Result<Option<[i32; N]>> {
        let Some(values) = self.int_vector(name)? else {
            return Ok(None);
        };
        <[i32; N]>::try_from(values.as_slice())
            .map(Some)
            .map_err(|_| self.wrong_count(name, ValueCount::Exactly(N), values.len()))
    }

    fn real_token(&self, name: &str, token: &str) -> Result<f64> {
        let x: f64 = token.parse().map_err(|_| self.bad_number(name, token))?;
        if x.is_finite() {
            if x.abs() < f64::MIN_POSITIVE && mantissa_is_nonzero(token) {
                return Err(MjcfError::out_of_range_in(&self.element, name, token));
            }
            return Ok(x);
        }
        // A literal that overflows (`1e400`) is MuJoCo's "number is too large";
        // a written NaN or infinity, which MuJoCo loads, is refused
        // (D-NONFINITE).
        if x.is_infinite() && !names_infinity(token) {
            return Err(MjcfError::out_of_range_in(&self.element, name, token));
        }
        Err(MjcfError::non_finite(&self.element, name, x))
    }

    pub(super) fn bad_number(&self, name: &str, value: &str) -> MjcfError {
        MjcfError::bad_number_in(&self.element, name, value)
    }

    pub(super) fn wrong_count(&self, name: &str, expected: ValueCount, found: usize) -> MjcfError {
        MjcfError::wrong_count_in(&self.element, name, expected, found)
    }
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
        Err(MjcfError::invalid_value(
            "mesh",
            "maxhullvert",
            "maxhullvert must be larger than 3",
        ))
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
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(())
}
