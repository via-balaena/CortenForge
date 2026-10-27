//! `<actuator>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{MjcfActuator, MjcfActuatorType, Prefix};

use super::attrs::{Attrs, skip_element};
use super::extension::{parse_plugin_ref, parse_plugin_ref_empty};

/// Parse actuators element.
pub(super) fn parse_actuators<R: BufRead>(reader: &mut Reader<R>) -> Result<Vec<MjcfActuator>> {
    let mut actuators = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(actuator_type) = MjcfActuatorType::from_str(&elem_name) {
                    let mut actuator = parse_actuator_attrs(e, actuator_type)?;
                    // Parse children (may contain <plugin> sub-element)
                    parse_actuator_children(reader, elem_name.as_bytes(), &mut actuator)?;
                    actuators.push(actuator);
                }
            }
            Ok(Event::Empty(ref e)) => {
                let elem_name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                if let Some(actuator_type) = MjcfActuatorType::from_str(&elem_name) {
                    let actuator = parse_actuator_attrs(e, actuator_type)?;
                    actuators.push(actuator);
                    // Self-closing, no need to skip
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

    actuator.ctrllimited = a.is_true("ctrllimited");
    actuator.forcelimited = a.is_true("forcelimited");

    if let Some(kp) = a.real("kp")? {
        actuator.kp = kp;
    }
    if let Some(kv) = a.real("kv")? {
        actuator.kv = Some(kv);
    }
    if let Some(dampratio) = a.real("dampratio")? {
        actuator.dampratio = Some(dampratio);
    }

    // Interpolation attributes (common to all actuator types)
    actuator.nsample = a.int("nsample")?;
    actuator.interp = a.string("interp");
    actuator.delay = a.real("delay")?;

    // ========================================================================
    // <general>-specific attributes
    // ========================================================================
    // These are only parsed for <general> actuators. For shortcut types,
    // these attributes are not part of the MJCF schema and are ignored.
    if actuator_type == MjcfActuatorType::General {
        actuator.gaintype = a.string("gaintype");
        actuator.biastype = a.string("biastype");
        actuator.dyntype = a.string("dyntype");
        actuator.gainprm = gain_or_bias(&a, "gainprm")?;
        actuator.biasprm = gain_or_bias(&a, "biasprm")?;
        actuator.dynprm = a.prefix("dynprm")?;
    }

    // ========================================================================
    // Cylinder-specific attributes
    // ========================================================================
    if let Some(area) = a.real("area")? {
        actuator.area = area;
    }
    if let Some(diameter) = a.real("diameter")? {
        actuator.diameter = Some(diameter);
    }
    // A muscle's `timeconst` is a pair [activation, deactivation]; every other
    // type's is one number (`xml_native_reader.cc` reads 2 and 1, exactly).
    if actuator_type == MjcfActuatorType::Muscle {
        if let Some(pair) = a.array::<2>("timeconst")? {
            actuator.muscle_timeconst = pair.into();
        }
    } else if let Some(timeconst) = a.real("timeconst")? {
        actuator.timeconst = Some(timeconst);
    }
    if let Some(bias) = a.array::<3>("bias")? {
        actuator.bias = bias;
    }

    // ========================================================================
    // Muscle-specific attributes
    // ========================================================================
    if let Some(pair) = a.array::<2>("range")? {
        actuator.range = pair.into();
    }
    if let Some(force) = a.real("force")? {
        actuator.force = force;
    }
    if let Some(scale) = a.real("scale")? {
        actuator.scale = scale;
    }
    if let Some(lmin) = a.real("lmin")? {
        actuator.lmin = lmin;
    }
    if let Some(lmax) = a.real("lmax")? {
        actuator.lmax = lmax;
    }
    if let Some(vmax) = a.real("vmax")? {
        actuator.vmax = vmax;
    }
    if let Some(fpmax) = a.real("fpmax")? {
        actuator.fpmax = fpmax;
    }
    if let Some(fvmax) = a.real("fvmax")? {
        actuator.fvmax = fvmax;
    }

    // ========================================================================
    // Adhesion-specific attributes
    // ========================================================================
    if let Some(gain) = a.real("gain")? {
        actuator.gain = gain;
    }

    // ========================================================================
    // Activation parameters
    // ========================================================================
    actuator.group = a.int("group")?;
    actuator.actlimited = a.is_true("actlimited");
    if let Some(pair) = a.array::<2>("actrange")? {
        actuator.actrange = Some(pair.into());
    }
    actuator.actearly = a.is_true("actearly");
    if let Some(pair) = a.array::<2>("lengthrange")? {
        actuator.lengthrange = Some(pair.into());
    }
    if let Some(user) = a.vector("user")? {
        actuator.user = user;
    }

    Ok(actuator)
}

/// Parse children of an actuator element (may contain `<plugin>` sub-element).
pub(super) fn parse_actuator_children<R: BufRead>(
    reader: &mut Reader<R>,
    parent_tag: &[u8],
    actuator: &mut MjcfActuator,
) -> Result<()> {
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_ref = parse_plugin_ref(reader, e)?;
                actuator.plugin = Some(plugin_ref);
            }
            Ok(Event::Empty(ref e)) if e.name().as_ref() == b"plugin" => {
                let plugin_ref = parse_plugin_ref_empty(e)?;
                actuator.plugin = Some(plugin_ref);
            }
            Ok(Event::Start(ref e)) => {
                // Unknown child — skip
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
