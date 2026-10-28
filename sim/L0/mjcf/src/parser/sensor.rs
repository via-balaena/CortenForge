//! `<sensor>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{MjcfSensor, MjcfSensorType};

use super::attrs::{Attrs, is_c_space, skip_element};
use super::extension::{parse_plugin_ref, parse_plugin_ref_empty};

// ============================================================================
// Sensor parsing
// ============================================================================

/// Parse the sensor section.
pub(super) fn parse_sensors<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfSensor>> {
    let mut sensors = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                let sensor_type = MjcfSensorType::from_str(&elem_name)
                    .ok_or_else(|| unread_sensor(&elem_name))?;
                let mut sensor = parse_sensor_attrs(e, sensor_type)?;
                // Parse children (may contain <plugin> sub-element)
                parse_sensor_children(reader, elem_name.as_bytes(), &mut sensor)?;
                sensors.push(sensor);
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                let sensor_type = MjcfSensorType::from_str(&elem_name)
                    .ok_or_else(|| unread_sensor(&elem_name))?;
                let sensor = parse_sensor_attrs(e, sensor_type)?;
                sensors.push(sensor);
                // Self-closing, no need to skip
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"sensor" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("sensor")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(sensors)
}

/// The refusal of an element in `<sensor>` that is not a sensor sim-mjcf
/// reads: a sensor MuJoCo 3.5.0 makes and sim-mjcf cannot (its schema,
/// `xml_native_reader.cc:446-503`; registry D-UNREAD-SENSOR), or any other
/// element, which MuJoCo's schema refuses as unrecognized.
fn unread_sensor(element: &str) -> MjcfError {
    const MUJOCO_SENSORS: [&str; 11] = [
        "camprojection",
        "contact",
        "e_kinetic",
        "e_potential",
        "insidesite",
        "jointlimitpos",
        "jointlimitvel",
        "plugin",
        "tactile",
        "tendonlimitpos",
        "tendonlimitvel",
    ];
    if MUJOCO_SENSORS.contains(&element) {
        MjcfError::unsupported(format!("the <{element}> sensor"))
    } else {
        MjcfError::unknown_element(element, "sensor")
    }
}

/// A rangefinder's `data`, read as MuJoCo 3.5.0 reads it
/// (`xml_native_reader.cc:4104-4124`, `MapValues` at `xml_util.cc:953-984`):
/// words of `raydata_map` separated by space, none twice, then in the map's
/// order; unwritten or empty, `dist`. sim-core computes a rangefinder's
/// distance only, so any other word is not supported (D-RANGEFINDER-DATA).
fn check_ray_data(a: &Attrs) -> Result<()> {
    const WORDS: [&str; 6] = ["dist", "dir", "origin", "point", "normal", "depth"];
    let Some(text) = a.text("data") else {
        return Ok(());
    };
    let mut keys = Vec::new();
    for word in text.split(is_c_space).filter(|w| !w.is_empty()) {
        let key = WORDS
            .iter()
            .position(|w| *w == word)
            .ok_or_else(|| MjcfError::invalid_keyword("rangefinder", "data", word))?;
        if keys.contains(&key) {
            return Err(MjcfError::invalid_value(
                "rangefinder",
                "data",
                format!("duplicate keyword: '{word}'"),
            ));
        }
        keys.push(key);
    }
    if keys.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err(MjcfError::invalid_value(
            "rangefinder",
            "data",
            format!("data attributes must be in order: {}", WORDS.join(", ")),
        ));
    }
    if keys.iter().any(|&key| key != 0) {
        return Err(MjcfError::unsupported(format!(
            "a rangefinder's data \"{text}\""
        )));
    }
    Ok(())
}

/// Parse sensor attributes.
pub(super) fn parse_sensor_attrs(
    e: &BytesStart,
    sensor_type: MjcfSensorType,
) -> Result<MjcfSensor> {
    let a = Attrs::new(e)?;
    let mut sensor = MjcfSensor {
        sensor_type,
        ..Default::default()
    };

    sensor.name = a.string("name").unwrap_or_default();
    sensor.class = a.string("class");

    // Different sensor types use different attribute names for their target
    sensor.objname = a
        .string("joint")
        .or_else(|| a.string("site"))
        .or_else(|| a.string("body"))
        .or_else(|| a.string("geom"))
        .or_else(|| a.string("tendon"))
        .or_else(|| a.string("actuator"))
        .or_else(|| a.string("objname"));

    // Dual-object attributes for distance/normal/fromto sensors
    if matches!(
        sensor_type,
        MjcfSensorType::Distance | MjcfSensorType::Normal | MjcfSensorType::Fromto
    ) {
        sensor.geom1 = a.string("geom1");
        sensor.geom2 = a.string("geom2");
        sensor.body1 = a.string("body1");
        sensor.body2 = a.string("body2");

        // Strict XOR validation: exactly one of {geom1, body1} required
        let has_obj1 = sensor.geom1.is_some() || sensor.body1.is_some();
        let has_both_obj1 = sensor.geom1.is_some() && sensor.body1.is_some();
        if !has_obj1 || has_both_obj1 {
            tracing::warn!(
                "sensor '{}': exactly one of (geom1, body1) must be specified",
                sensor.name
            );
        }

        // Strict XOR validation: exactly one of {geom2, body2} required
        let has_obj2 = sensor.geom2.is_some() || sensor.body2.is_some();
        let has_both_obj2 = sensor.geom2.is_some() && sensor.body2.is_some();
        if !has_obj2 || has_both_obj2 {
            tracing::warn!(
                "sensor '{}': exactly one of (geom2, body2) must be specified",
                sensor.name
            );
        }
    }

    // Explicit object type (frame sensors only — builder validates)
    sensor.objtype = a.string("objtype");

    // Separate reftype and refname (previously conflated)
    sensor.reftype = a.string("reftype");
    sensor.refname = a.string("refname");

    if let Some(noise) = a.real("noise")? {
        sensor.noise = noise;
    }

    if let Some(cutoff) = a.real("cutoff")? {
        sensor.cutoff = cutoff;
    }

    if let Some(user) = a.vector("user")? {
        sensor.user = user;
    }

    sensor.nsample = a.int("nsample")?;
    sensor.interp = a.keyword_text("interp", keywords::interp)?;
    sensor.delay = a.real("delay")?;
    // MuJoCo reads `period [phase]` (`exact = false`); the phase is read with
    // the rest of delay and history (book L48), the period here.
    sensor.interval = a
        .prefix::<2>("interval")?
        .map(|interval| interval.given()[0]);

    if sensor_type == MjcfSensorType::Rangefinder {
        check_ray_data(&a)?;
    }
    if sensor_type == MjcfSensorType::User {
        sensor.dim = a.int("dim")?;
        sensor.needstage = a.keyword_text("needstage", keywords::sensor_stage)?;
        sensor.datatype = a.keyword_text("datatype", keywords::sensor_datatype)?;
    }

    Ok(sensor)
}

/// Parse children of a sensor element (may contain `<plugin>` sub-element).
pub(super) fn parse_sensor_children<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
    sensor: &mut MjcfSensor,
) -> Result<()> {
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_ref = parse_plugin_ref(reader, e)?;
                sensor.plugin = Some(plugin_ref);
            }
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_ref = parse_plugin_ref_empty(e)?;
                sensor.plugin = Some(plugin_ref);
            }
            Ok(Event::Start(ref e)) => {
                let name = e.name().as_ref().to_vec();
                skip_element(reader, &name)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == parent_tag => break,
            Ok(Event::Eof) => {
                return Err(MjcfError::eof_in(String::from_utf8_lossy(parent_tag)));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }
    Ok(())
}
