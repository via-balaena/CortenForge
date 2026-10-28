//! `<sensor>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{MjcfSensor, MjcfSensorType};

use super::attrs::{Attrs, skip_element};
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
                if let Some(sensor_type) = MjcfSensorType::from_str(&elem_name) {
                    let mut sensor = parse_sensor_attrs(e, sensor_type)?;
                    // Parse children (may contain <plugin> sub-element)
                    parse_sensor_children(reader, elem_name.as_bytes(), &mut sensor)?;
                    sensors.push(sensor);
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(sensor_type) = MjcfSensorType::from_str(&elem_name) {
                    let sensor = parse_sensor_attrs(e, sensor_type)?;
                    sensors.push(sensor);
                    // Self-closing, no need to skip
                }
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
