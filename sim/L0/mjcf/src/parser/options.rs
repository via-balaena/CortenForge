//! `<option>`, `<compiler>`, `<size>`, and `<flag>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};
use crate::keywords;

use crate::types::{
    MjcfCompiler, MjcfConeType, MjcfFlag, MjcfIntegrator, MjcfJacobianType, MjcfModel, MjcfOption,
    MjcfSolverType,
};

use super::attrs::{Attrs, skip_element};
use crate::error::ValueCount;

/// Parse `<size>` element attributes into MjcfModel nuser_* fields.
/// Multiple `<size>` elements merge with last-writer-wins per attribute.
pub(super) fn parse_size_attrs(e: &BytesStart, model: &mut MjcfModel) -> Result<()> {
    let a = Attrs::new(e)?;
    if let Some(val) = a.int("nuser_body")? {
        model.nuser_body = val;
    }
    if let Some(val) = a.int("nuser_jnt")? {
        model.nuser_jnt = val;
    }
    if let Some(val) = a.int("nuser_geom")? {
        model.nuser_geom = val;
    }
    if let Some(val) = a.int("nuser_site")? {
        model.nuser_site = val;
    }
    if let Some(val) = a.int("nuser_tendon")? {
        model.nuser_tendon = val;
    }
    if let Some(val) = a.int("nuser_actuator")? {
        model.nuser_actuator = val;
    }
    if let Some(val) = a.int("nuser_sensor")? {
        model.nuser_sensor = val;
    }
    Ok(())
}

/// An `<option>` element, read onto `option`: MuJoCo reads every `<option>`,
/// and the one `<flag>` it may hold, onto the same values, each setting only
/// what it writes (`xml_native_reader.cc`, the Option loop; a second `<flag>`
/// is a schema error, book L47's).
pub(super) fn parse_option<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
    option: &mut MjcfOption,
    empty: bool,
) -> Result<()> {
    parse_option_attrs(start, option)?;
    if empty {
        return Ok(());
    }
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(ref event @ (Event::Start(ref e) | Event::Empty(ref e))) => {
                let child_empty = matches!(event, Event::Empty(_));
                // Handle flag element
                if e.name().as_ref() == b"flag" {
                    parse_flag_attrs(e, &mut option.flag)?;
                }
                skip_element(reader, e.name().as_ref(), child_empty)?;
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"option" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("option")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(())
}

/// An `<option>`'s attributes, read onto `option`.
fn parse_option_attrs(e: &BytesStart, option: &mut MjcfOption) -> Result<()> {
    let a = Attrs::new(e)?;

    // Core simulation
    if let Some(ts) = a.real("timestep")? {
        option.timestep = ts;
    }
    if let Some(integrator) = a.keyword("integrator", MjcfIntegrator::from_str)? {
        option.integrator = integrator;
    }

    // Solver configuration
    if let Some(solver) = a.keyword("solver", MjcfSolverType::from_str)? {
        option.solver = solver;
    }
    if let Some(iter) = a.int("iterations")? {
        option.iterations = iter.max(0) as usize;
    }
    if let Some(tol) = a.real("tolerance")? {
        option.tolerance = tol;
    }
    if let Some(ls_iter) = a.int("ls_iterations")? {
        option.ls_iterations = ls_iter.max(0) as usize;
    }
    if let Some(ls_tol) = a.real("ls_tolerance")? {
        option.ls_tolerance = ls_tol;
    }
    if let Some(noslip) = a.int("noslip_iterations")? {
        option.noslip_iterations = noslip.max(0) as usize;
    }
    if let Some(noslip_tol) = a.real("noslip_tolerance")? {
        option.noslip_tolerance = noslip_tol;
    }
    if let Some(ccd) = a.int("ccd_iterations")? {
        option.ccd_iterations = ccd.max(0) as usize;
    }
    if let Some(ccd_tol) = a.real("ccd_tolerance")? {
        option.ccd_tolerance = ccd_tol;
    }
    if let Some(sdf_iter) = a.int("sdf_iterations")? {
        option.sdf_iterations = sdf_iter.max(0) as usize;
    }
    if let Some(sdf_init) = a.int("sdf_initpoints")? {
        option.sdf_initpoints = sdf_init.max(0) as usize;
    }

    // Contact configuration
    if let Some(cone) = a.keyword("cone", MjcfConeType::from_str)? {
        option.cone = cone;
    }
    if let Some(jacobian) = a.keyword("jacobian", MjcfJacobianType::from_str)? {
        option.jacobian = jacobian;
    }
    if let Some(impratio) = a.real("impratio")? {
        option.impratio = impratio;
    }
    if let Some(regularization) = a.real("regularization")? {
        option.regularization = regularization;
    }
    if let Some(fsmooth) = a.real("friction_smoothing")? {
        option.friction_smoothing = fsmooth;
    }

    // Physics environment
    if let Some(grav) = a.vec3("gravity")? {
        option.gravity = grav;
    }
    if let Some(wind) = a.vec3("wind")? {
        option.wind = wind;
    }
    if let Some(magnetic) = a.vec3("magnetic")? {
        option.magnetic = magnetic;
    }
    if let Some(density) = a.real("density")? {
        option.density = density;
    }
    if let Some(viscosity) = a.real("viscosity")? {
        option.viscosity = viscosity;
    }

    // Constraint limits
    if let Some(nconmax) = a.int("nconmax")? {
        option.nconmax = nconmax.max(0) as usize;
    }
    if let Some(njmax) = a.int("njmax")? {
        option.njmax = njmax.max(0) as usize;
    }

    // Overrides
    if let Some(o_margin) = a.real("o_margin")? {
        option.o_margin = o_margin;
    }
    // A later <option>'s prefix lies over an earlier one's values.
    if let Some(given) = a.prefix("o_solimp")? {
        option.o_solimp = Some(option.o_solimp.map_or(given, |was| given.over_prefix(&was)));
    }
    if let Some(given) = a.prefix("o_solref")? {
        option.o_solref = Some(option.o_solref.map_or(given, |was| given.over_prefix(&was)));
    }
    if let Some(given) = a.prefix("o_friction")? {
        option.o_friction = Some(
            option
                .o_friction
                .map_or(given, |was| given.over_prefix(&was)),
        );
    }

    // Sleep
    if let Some(st) = a.real("sleep_tolerance")? {
        option.sleep_tolerance = st;
    }

    // Per-group actuator disabling: up to 31 groups, each in 0..=30
    // (`xml_native_reader.cc` reads 31 with `exact = false` and refuses a
    // group outside that range).
    if let Some(groups) = a.int_vector("actuatorgroupdisable")? {
        if groups.len() > 31 {
            return Err(a.wrong_count(
                "actuatorgroupdisable",
                ValueCount::AtMost(31),
                groups.len(),
            ));
        }
        for group in groups {
            let bit = u32::try_from(group)
                .ok()
                .filter(|bit| *bit <= 30)
                .ok_or_else(|| {
                    MjcfError::invalid_value(
                        "option",
                        "actuatorgroupdisable",
                        format!("group {group} is not in 0..=30"),
                    )
                })?;
            option.actuatorgroupdisable |= 1u32 << bit;
        }
    }

    Ok(())
}

/// A `<compiler>`'s attributes, read onto `compiler`: MuJoCo reads every
/// `<compiler>` onto the same settings, each setting only what it writes.
pub(super) fn parse_compiler_attrs(e: &BytesStart, compiler: &mut MjcfCompiler) -> Result<()> {
    let a = Attrs::new(e)?;

    // A1. angle — angular unit
    if let Some(angle) = a.keyword("angle", keywords::angle)? {
        compiler.angle = angle;
    }

    // A2. eulerseq — Euler rotation sequence
    if let Some(seq) = a.text("eulerseq") {
        if seq.len() != 3
            || !seq
                .chars()
                .all(|c| matches!(c, 'x' | 'y' | 'z' | 'X' | 'Y' | 'Z'))
        {
            return Err(MjcfError::invalid_value(
                "compiler",
                "eulerseq",
                format!("expected 3 characters from {{x,y,z,X,Y,Z}}, got '{seq}'"),
            ));
        }
        seq.clone_into(&mut compiler.eulerseq);
    }

    // A3. assetdir / meshdir / texturedir — asset path resolution. MuJoCo
    // writes assetdir into meshdir and texturedir as it reads it, then reads
    // meshdir and texturedir over it (`xml_native_reader.cc`, Compiler), so a
    // later <compiler>'s assetdir lies over an earlier one's meshdir.
    if let Some(assetdir) = a.string("assetdir") {
        compiler.meshdir = Some(assetdir.clone());
        compiler.texturedir = Some(assetdir.clone());
        compiler.assetdir = Some(assetdir);
    }
    if let Some(meshdir) = a.string("meshdir") {
        compiler.meshdir = Some(meshdir);
    }
    if let Some(texturedir) = a.string("texturedir") {
        compiler.texturedir = Some(texturedir);
    }

    // A4. autolimits
    if let Some(autolimits) = a.boolean("autolimits")? {
        compiler.autolimits = autolimits;
    }

    // A5. inertiafromgeom
    if let Some(ifg) = a.keyword("inertiafromgeom", keywords::inertia_from_geom)? {
        compiler.inertiafromgeom = ifg;
    }

    // A6. boundmass / boundinertia
    if let Some(bm) = a.real("boundmass")? {
        compiler.boundmass = bm;
    }
    if let Some(bi) = a.real("boundinertia")? {
        compiler.boundinertia = bi;
    }

    // A7. balanceinertia
    if let Some(balance) = a.boolean("balanceinertia")? {
        compiler.balanceinertia = balance;
    }

    // A8. settotalmass
    if let Some(stm) = a.real("settotalmass")? {
        compiler.settotalmass = stm;
    }

    // A9. strippath
    if let Some(sp) = a.boolean("strippath")? {
        compiler.strippath = sp;
    }

    // A10. discardvisual
    if let Some(dv) = a.boolean("discardvisual")? {
        compiler.discardvisual = dv;
    }

    // A11. fusestatic
    if let Some(fs) = a.boolean("fusestatic")? {
        compiler.fusestatic = fs;
    }

    // A12. coordinate (deprecated — reject "global")
    if a.keyword("coordinate", keywords::coordinate_is_global)? == Some(true) {
        return Err(MjcfError::invalid_value(
            "compiler",
            "coordinate",
            "coordinate='global' was removed in MuJoCo 2.3.4",
        ));
    }

    // Deferred attributes (stored but no behavior yet)
    if let Some(fitaabb) = a.boolean("fitaabb")? {
        compiler.fitaabb = fitaabb;
    }
    if let Some(usethread) = a.boolean("usethread")? {
        compiler.usethread = usethread;
    }
    if let Some(alignfree) = a.boolean("alignfree")? {
        compiler.alignfree = alignfree;
    }

    // A13. exactmeshinertia: not a MuJoCo 3.5.0 attribute (its schema refuses
    // it, book L47); read, warned about and given no effect.
    if let Some(emi) = a.boolean("exactmeshinertia")? {
        compiler.exactmeshinertia = emi;
        tracing::warn!(
            "exactmeshinertia is deprecated and has no effect; \
             use <mesh inertia=\"exact\"/> instead"
        );
    }

    Ok(())
}

/// A `<flag>`'s attributes, read onto `flag`: each flag it does not write
/// keeps its value.
pub(super) fn parse_flag_attrs(e: &BytesStart, flag: &mut MjcfFlag) -> Result<()> {
    let a = Attrs::new(e)?;

    // Each flag is MuJoCo's `enable_map` keyword: exactly `disable` or
    // `enable`.
    let parse_flag = |name: &str, current: bool| -> Result<bool> {
        Ok(a.keyword(name, |s| match s {
            "disable" => Some(false),
            "enable" => Some(true),
            _ => None,
        })?
        .unwrap_or(current))
    };

    // Disable flags (true = enabled, false = disabled)
    flag.constraint = parse_flag("constraint", flag.constraint)?;
    flag.equality = parse_flag("equality", flag.equality)?;
    flag.frictionloss = parse_flag("frictionloss", flag.frictionloss)?;
    flag.limit = parse_flag("limit", flag.limit)?;
    flag.contact = parse_flag("contact", flag.contact)?;
    // MuJoCo 3.3.6+ split `passive` into independent `spring` + `damper`.
    // `passive` is silently ignored (unrecognized attributes are skipped).
    flag.spring = parse_flag("spring", flag.spring)?;
    flag.damper = parse_flag("damper", flag.damper)?;
    flag.gravity = parse_flag("gravity", flag.gravity)?;
    flag.clampctrl = parse_flag("clampctrl", flag.clampctrl)?;
    flag.warmstart = parse_flag("warmstart", flag.warmstart)?;
    flag.filterparent = parse_flag("filterparent", flag.filterparent)?;
    flag.actuation = parse_flag("actuation", flag.actuation)?;
    flag.refsafe = parse_flag("refsafe", flag.refsafe)?;
    flag.sensor = parse_flag("sensor", flag.sensor)?;
    flag.midphase = parse_flag("midphase", flag.midphase)?;
    flag.nativeccd = parse_flag("nativeccd", flag.nativeccd)?;
    flag.eulerdamp = parse_flag("eulerdamp", flag.eulerdamp)?;
    flag.autoreset = parse_flag("autoreset", flag.autoreset)?;
    flag.island = parse_flag("island", flag.island)?;
    // Enable flags (false = disabled, true = enabled)
    flag.override_contacts = parse_flag("override", flag.override_contacts)?;
    flag.energy = parse_flag("energy", flag.energy)?;
    flag.fwdinv = parse_flag("fwdinv", flag.fwdinv)?;
    flag.invdiscrete = parse_flag("invdiscrete", flag.invdiscrete)?;
    flag.multiccd = parse_flag("multiccd", flag.multiccd)?;
    flag.sleep = parse_flag("sleep", flag.sleep)?;

    Ok(())
}
