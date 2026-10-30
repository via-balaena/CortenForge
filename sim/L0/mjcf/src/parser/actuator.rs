//! `<actuator>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{MjcfActuator, MjcfActuatorType, Prefix};

use super::attrs::{Attrs, skip_element};
use super::extension::parse_plugin_ref;

/// Parse actuators element (`empty`: it closed itself, so it has none).
pub(super) fn parse_actuators<R: BufRead>(
    reader: &mut Reader<R>,
    empty: bool,
) -> Result<Vec<MjcfActuator>> {
    let mut actuators = Vec::new();
    if empty {
        return Ok(actuators);
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(actuator_type) = MjcfActuatorType::from_str(&elem_name) {
                    let mut actuator = parse_actuator_attrs(e, actuator_type)?;
                    // Parse children (may contain <plugin> sub-element)
                    parse_actuator_children(
                        reader,
                        elem_name.as_bytes(),
                        &mut actuator,
                        child_empty,
                    )?;
                    actuators.push(actuator);
                } else {
                    skip_element(reader, e.name().as_ref(), child_empty)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"actuator" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("actuator")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(actuators)
}

/// An actuator's `actdim` is not read: the builder gives a stateful actuator
/// one activation, so a written count, which MuJoCo honours for `user`
/// dynamics, is refused rather than ignored.
pub(super) fn refuse_actdim(a: &Attrs) -> Result<()> {
    if a.has("actdim") {
        return Err(MjcfError::unsupported("an actuator's actdim"));
    }
    Ok(())
}

/// A servo's `inheritrange`, read only where MuJoCo 3.5.0's schema has it:
/// on a `<position>` or `<intvelocity>`, as an element or a default class's
/// (`xml_native_reader.cc:189-196`, `:402-416`); elsewhere refused, as its
/// schema refuses it. An intvelocity's schema has no `timeconst` and no
/// `actlimited` (its activation is always limited); both are refused.
pub(super) fn servo_attributes(a: &Attrs) -> Result<Option<f64>> {
    let element = a.element();
    if element == "intvelocity" {
        for attribute in ["timeconst", "actlimited"] {
            if a.has(attribute) {
                return Err(MjcfError::unknown_attribute(element, attribute));
            }
        }
    }
    if matches!(element, "position" | "intvelocity") {
        a.real("inheritrange")
    } else if a.has("inheritrange") {
        Err(MjcfError::unknown_attribute(element, "inheritrange"))
    } else {
        Ok(None)
    }
}

/// `gainprm` or `biasprm`: MuJoCo 3.5.0 reads up to ten (`mjNGAIN`,
/// `mjNBIAS`); sim-core stores nine. A tenth equal to MuJoCo's default, 0, is
/// read as unwritten; any other is refused (D-GAINPRM-10).
pub(super) fn gain_or_bias(a: &Attrs, name: &str) -> Result<Option<Prefix<9>>> {
    let Some(given) = a.prefix::<10>(name)? else {
        return Ok(None);
    };
    let given = given.given();
    if let [nine @ .., tenth] = given
        && given.len() == 10
    {
        if *tenth != 0.0 {
            return Err(MjcfError::unsupported(format!(
                "a nonzero tenth {name} value on <{}>",
                a.element()
            )));
        }
        return Ok(Prefix::new(nine));
    }
    Ok(Prefix::new(given))
}

/// Parse actuator attributes.
pub(super) fn parse_actuator_attrs(
    e: &BytesStart,
    actuator_type: MjcfActuatorType,
) -> Result<MjcfActuator> {
    let a = Attrs::new(e)?;
    refuse_actdim(&a)?;
    let mut actuator = MjcfActuator {
        actuator_type,
        ..Default::default()
    };

    // Common attributes
    actuator.name = a.string("name").unwrap_or_default();
    actuator.class = a.string("class");
    actuator.joint = a.string("joint");
    actuator.site = a.string("site");
    actuator.tendon = a.string("tendon");
    actuator.body = a.string("body");

    actuator.gear = a.prefix("gear")?;
    actuator.refsite = a.string("refsite");
    actuator.jointinparent = a.string("jointinparent");
    actuator.cranksite = a.string("cranksite");
    actuator.slidersite = a.string("slidersite");
    actuator.cranklength = a.real("cranklength")?;

    if let Some(pair) = a.array::<2>("ctrlrange")? {
        actuator.ctrlrange = Some(pair.into());
    }
    if let Some(pair) = a.array::<2>("forcerange")? {
        actuator.forcerange = Some(pair.into());
    }

    actuator.ctrllimited = a.tf_auto("ctrllimited")?;
    actuator.forcelimited = a.tf_auto("forcelimited")?;

    actuator.kp = a.real("kp")?;
    if let Some(kv) = a.real("kv")? {
        actuator.kv = Some(kv);
    }
    if let Some(dampratio) = a.real("dampratio")? {
        actuator.dampratio = Some(dampratio);
    }
    actuator.inheritrange = servo_attributes(&a)?;

    // Interpolation attributes (common to all actuator types)
    actuator.nsample = a.int("nsample")?;
    actuator.interp = a.keyword_text("interp", keywords::interp)?;
    actuator.delay = a.real("delay")?;

    // ========================================================================
    // <general>-specific attributes
    // ========================================================================
    // These are only parsed for <general> actuators. For shortcut types,
    // these attributes are not part of the MJCF schema and are ignored.
    if actuator_type == MjcfActuatorType::General {
        actuator.gaintype = a.keyword_text("gaintype", keywords::gain_type)?;
        actuator.biastype = a.keyword_text("biastype", keywords::bias_type)?;
        actuator.dyntype = a.keyword_text("dyntype", keywords::dyn_type)?;
        actuator.gainprm = gain_or_bias(&a, "gainprm")?;
        actuator.biasprm = gain_or_bias(&a, "biasprm")?;
        actuator.dynprm = a.prefix("dynprm")?;
    }

    // ========================================================================
    // Cylinder-specific attributes
    // ========================================================================
    actuator.area = a.real("area")?;
    if let Some(diameter) = a.real("diameter")? {
        actuator.diameter = Some(diameter);
    }
    // A muscle's `timeconst` is a pair [activation, deactivation]; every other
    // type's is one number (`xml_native_reader.cc` reads 2 and 1, exactly).
    if actuator_type == MjcfActuatorType::Muscle {
        actuator.muscle_timeconst = a.array::<2>("timeconst")?.map(Into::into);
    } else if let Some(timeconst) = a.real("timeconst")? {
        actuator.timeconst = Some(timeconst);
    }
    actuator.bias = a.array::<3>("bias")?;

    // ========================================================================
    // Muscle-specific attributes
    // ========================================================================
    actuator.range = a.array::<2>("range")?.map(Into::into);
    actuator.force = a.real("force")?;
    actuator.scale = a.real("scale")?;
    actuator.lmin = a.real("lmin")?;
    actuator.lmax = a.real("lmax")?;
    actuator.vmax = a.real("vmax")?;
    actuator.fpmax = a.real("fpmax")?;
    actuator.fvmax = a.real("fvmax")?;

    // ========================================================================
    // Adhesion-specific attributes
    // ========================================================================
    actuator.gain = a.real("gain")?;

    // ========================================================================
    // Activation parameters
    // ========================================================================
    actuator.group = a.int("group")?;
    actuator.actlimited = a.tf_auto("actlimited")?;
    if let Some(pair) = a.array::<2>("actrange")? {
        actuator.actrange = Some(pair.into());
    }
    actuator.actearly = a.boolean("actearly")?;
    if let Some(pair) = a.array::<2>("lengthrange")? {
        actuator.lengthrange = Some(pair.into());
    }
    if let Some(user) = a.vector("user")? {
        actuator.user = user;
    }
    crate::builder::actuator::check_servo_gains(
        a.element(),
        actuator.kv,
        actuator.dampratio,
        actuator.timeconst,
    )?;
    crate::builder::actuator::check_cylinder_bias(a.element(), actuator.bias)?;
    crate::builder::actuator::check_muscle_values(
        a.element(),
        actuator.muscle_timeconst,
        actuator.range,
        [
            actuator.scale,
            actuator.lmin,
            actuator.lmax,
            actuator.vmax,
            actuator.fpmax,
            actuator.fvmax,
        ],
    )?;

    Ok(actuator)
}

/// Parse children of an actuator element (may contain `<plugin>` sub-element;
/// `empty`: the actuator closed itself, so it has none).
pub(super) fn parse_actuator_children<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
    actuator: &mut MjcfActuator,
    empty: bool,
) -> Result<()> {
    if empty {
        return Ok(());
    }
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                if e.name().as_ref() == b"plugin" {
                    actuator.plugin = Some(parse_plugin_ref(reader, e, child_empty)?);
                } else {
                    // Unknown child — skip
                    skip_element(reader, e.name().as_ref(), child_empty)?;
                }
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
