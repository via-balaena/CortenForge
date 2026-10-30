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

/// An `<intvelocity>`'s `actlimited`, refused as MuJoCo 3.5.0's schema refuses
/// it (`xml_native_reader.cc:189-196`, `:402-416`): its activation is always
/// limited. The kind-specific attributes a kind does not read are refused
/// when it is written (`ActuatorSpec::write`).
pub(super) fn refuse_intvelocity_actlimited(a: &Attrs) -> Result<()> {
    if a.element() == "intvelocity" && a.has("actlimited") {
        return Err(MjcfError::unknown_attribute("intvelocity", "actlimited"));
    }
    Ok(())
}

/// `gainprm` or `biasprm`: MuJoCo 3.5.0 reads up to ten (`mjNGAIN`,
/// `mjNBIAS`); sim-core stores nine. A tenth equal to MuJoCo's default, 0, is
/// read as unwritten; any other is refused on a `<general>` (D-GAINPRM-10),
/// and on another kind left to the write, which refuses the attribute.
pub(super) fn gain_or_bias(a: &Attrs, name: &str) -> Result<Option<Prefix<9>>> {
    let Some(given) = a.prefix::<10>(name)? else {
        return Ok(None);
    };
    let given = given.given();
    if let [nine @ .., tenth] = given
        && given.len() == 10
    {
        if *tenth != 0.0 && a.element() == "general" {
            return Err(MjcfError::unsupported(format!(
                "a nonzero tenth {name} value on <{}>",
                a.element()
            )));
        }
        return Ok(Prefix::new(nine));
    }
    Ok(Prefix::new(given))
}

/// Parse actuator attributes: its name, class and target, then its values
/// ([`actuator_values`]).
pub(super) fn parse_actuator_attrs(
    e: &BytesStart,
    actuator_type: MjcfActuatorType,
) -> Result<MjcfActuator> {
    let a = Attrs::new(e)?;
    let mut actuator = actuator_values(&a, actuator_type)?;
    actuator.name = a.string("name").unwrap_or_default();
    actuator.class = a.string("class");
    actuator.joint = a.string("joint");
    actuator.site = a.string("site");
    actuator.tendon = a.string("tendon");
    actuator.body = a.string("body");
    actuator.refsite = a.string("refsite");
    actuator.jointinparent = a.string("jointinparent");
    actuator.cranksite = a.string("cranksite");
    actuator.slidersite = a.string("slidersite");
    Ok(actuator)
}

/// The values an actuator element writes, in a default class or not (MuJoCo
/// 3.5.0's default schema gives a class's element each of them, but no name,
/// class or target: `xml_native_reader.cc:184-207`), each `None` where not
/// written, every kind's read whatever the kind: `ActuatorSpec::write`
/// refuses one its kind does not read, and holds every value's rule. Refused
/// here are `actdim` and an intvelocity's `actlimited`.
pub(super) fn actuator_values(a: &Attrs, actuator_type: MjcfActuatorType) -> Result<MjcfActuator> {
    refuse_actdim(a)?;
    let mut actuator = MjcfActuator {
        actuator_type,
        ..Default::default()
    };
    actuator.gear = a.prefix("gear")?;
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
    actuator.kv = a.real("kv")?;
    actuator.dampratio = a.real("dampratio")?;
    refuse_intvelocity_actlimited(a)?;
    actuator.inheritrange = a.real("inheritrange")?;
    actuator.nsample = a.int("nsample")?;
    actuator.interp = a.keyword_text("interp", keywords::interp)?;
    actuator.delay = a.real("delay")?;

    // A <general>'s own: its types (their keywords checked on a <general>
    // alone: another kind writing one is refused when it is written,
    // `ActuatorSpec::write`), parameters and actearly
    // (`xml_native_reader.cc:2373-2391`).
    if actuator_type == MjcfActuatorType::General {
        actuator.gaintype = a.keyword_text("gaintype", keywords::gain_type)?;
        actuator.biastype = a.keyword_text("biastype", keywords::bias_type)?;
        actuator.dyntype = a.keyword_text("dyntype", keywords::dyn_type)?;
    } else {
        actuator.gaintype = a.string("gaintype");
        actuator.biastype = a.string("biastype");
        actuator.dyntype = a.string("dyntype");
    }
    actuator.gainprm = gain_or_bias(a, "gainprm")?;
    actuator.biasprm = gain_or_bias(a, "biasprm")?;
    actuator.dynprm = a.prefix("dynprm")?;
    actuator.actearly = a.boolean("actearly")?;
    actuator.tausmooth = a.real("tausmooth")?;

    // A cylinder's.
    actuator.area = a.real("area")?;
    actuator.diameter = a.real("diameter")?;
    // A muscle's `timeconst` is a pair [activation, deactivation]; every other
    // type's is one number (`xml_native_reader.cc` reads 2 and 1, exactly).
    if actuator_type == MjcfActuatorType::Muscle {
        actuator.muscle_timeconst = a.array::<2>("timeconst")?.map(Into::into);
    } else {
        actuator.timeconst = a.real("timeconst")?;
    }
    actuator.bias = a.array::<3>("bias")?;

    // A muscle's.
    actuator.range = a.array::<2>("range")?.map(Into::into);
    actuator.force = a.real("force")?;
    actuator.scale = a.real("scale")?;
    actuator.lmin = a.real("lmin")?;
    actuator.lmax = a.real("lmax")?;
    actuator.vmax = a.real("vmax")?;
    actuator.fpmax = a.real("fpmax")?;
    actuator.fvmax = a.real("fvmax")?;

    // An adhesion's.
    actuator.gain = a.real("gain")?;

    actuator.group = a.int("group")?;
    actuator.actlimited = a.tf_auto("actlimited")?;
    if let Some(pair) = a.array::<2>("actrange")? {
        actuator.actrange = Some(pair.into());
    }
    if let Some(pair) = a.array::<2>("lengthrange")? {
        actuator.lengthrange = Some(pair.into());
    }
    if let Some(user) = a.vector("user")? {
        actuator.user = user;
    }
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
