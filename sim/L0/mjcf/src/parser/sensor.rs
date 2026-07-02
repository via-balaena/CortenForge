//! `<sensor>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfSensor, MjcfSensorType};

use super::attrs::{
    get_attribute_opt, parse_float_array, parse_float_attr, parse_int_attr, skip_element,
};
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
                    let mut sensor = parse_sensor_attrs(e, sensor_type);
                    // Parse children (may contain <plugin> sub-element)
                    parse_sensor_children(reader, elem_name.as_bytes(), &mut sensor)?;
                    sensors.push(sensor);
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(sensor_type) = MjcfSensorType::from_str(&elem_name) {
                    let sensor = parse_sensor_attrs(e, sensor_type);
                    sensors.push(sensor);
                    // Self-closing, no need to skip
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"sensor" => break,
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in sensor".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(sensors)
}

/// Parse sensor attributes.
pub(super) fn parse_sensor_attrs(e: &BytesStart, sensor_type: MjcfSensorType) -> MjcfSensor {
    let mut sensor = MjcfSensor {
        sensor_type,
        ..Default::default()
    };

    sensor.name = get_attribute_opt(e, "name").unwrap_or_default();
    sensor.class = get_attribute_opt(e, "class");

    // Different sensor types use different attribute names for their target
    sensor.objname = get_attribute_opt(e, "joint")
        .or_else(|| get_attribute_opt(e, "site"))
        .or_else(|| get_attribute_opt(e, "body"))
        .or_else(|| get_attribute_opt(e, "geom"))
        .or_else(|| get_attribute_opt(e, "tendon"))
        .or_else(|| get_attribute_opt(e, "actuator"))
        .or_else(|| get_attribute_opt(e, "objname"));

    // Dual-object attributes for distance/normal/fromto sensors
    if matches!(
        sensor_type,
        MjcfSensorType::Distance | MjcfSensorType::Normal | MjcfSensorType::Fromto
    ) {
        sensor.geom1 = get_attribute_opt(e, "geom1");
        sensor.geom2 = get_attribute_opt(e, "geom2");
        sensor.body1 = get_attribute_opt(e, "body1");
        sensor.body2 = get_attribute_opt(e, "body2");

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
    sensor.objtype = get_attribute_opt(e, "objtype");

    // Separate reftype and refname (previously conflated)
    sensor.reftype = get_attribute_opt(e, "reftype");
    sensor.refname = get_attribute_opt(e, "refname");

    if let Some(noise) = parse_float_attr(e, "noise") {
        sensor.noise = noise;
    }

    if let Some(cutoff) = parse_float_attr(e, "cutoff") {
        sensor.cutoff = cutoff;
    }

    if let Some(user) = get_attribute_opt(e, "user") {
        if let Ok(parts) = parse_float_array(&user) {
            sensor.user = parts;
        }
    }

    sensor.nsample = parse_int_attr(e, "nsample");
    sensor.interp = get_attribute_opt(e, "interp");
    sensor.delay = parse_float_attr(e, "delay");
    sensor.interval = parse_float_attr(e, "interval");

    sensor
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
                return Err(MjcfError::XmlParse(format!(
                    "unexpected EOF in {}",
                    String::from_utf8_lossy(parent_tag)
                )));
            }
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }
    Ok(())
}
