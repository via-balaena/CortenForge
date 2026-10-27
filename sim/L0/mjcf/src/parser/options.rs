//! `<option>`, `<compiler>`, `<size>`, and `<flag>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    AngleUnit, InertiaFromGeom, MjcfCompiler, MjcfConeType, MjcfFlag, MjcfIntegrator,
    MjcfJacobianType, MjcfModel, MjcfOption, MjcfSolverType,
};

use super::attrs::Attrs;
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

/// Parse option element.
pub(super) fn parse_option<R: BufRead>(
    reader: &mut Reader<R>,
    start: &BytesStart,
) -> Result<MjcfOption> {
    let mut option = parse_option_attrs(start)?;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                // Handle flag element
                if e.name().as_ref() == b"flag" {
                    option.flag = parse_flag_attrs(e)?;
                }
            }
            Ok(Event::End(ref e)) if e.name().as_ref() == b"option" => break,
            Ok(Event::Eof) => return Err(MjcfError::eof_in("option")),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::xml(e)),
        }
        buf.clear();
    }

    Ok(option)
}

/// Parse option attributes only.
pub(super) fn parse_option_attrs(e: &BytesStart) -> Result<MjcfOption> {
    let a = Attrs::new(e)?;
    let mut option = MjcfOption::default();

    // Core simulation
    if let Some(ts) = a.real("timestep")? {
        option.timestep = ts;
    }
    if let Some(integrator) = a.text("integrator")
        && let Some(int) = MjcfIntegrator::from_str(integrator)
    {
        option.integrator = int;
    }

    // Solver configuration
    if let Some(solver) = a.text("solver")
        && let Some(s) = MjcfSolverType::from_str(solver)
    {
        option.solver = s;
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
    if let Some(cone) = a.text("cone")
        && let Some(c) = MjcfConeType::from_str(cone)
    {
        option.cone = c;
    }
    if let Some(jacobian) = a.text("jacobian")
        && let Some(j) = MjcfJacobianType::from_str(jacobian)
    {
        option.jacobian = j;
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
    option.o_solimp = a.prefix("o_solimp")?;
    option.o_solref = a.prefix("o_solref")?;
    option.o_friction = a.prefix("o_friction")?;

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

    Ok(option)
}

/// Parse compiler element attributes.
pub(super) fn parse_compiler_attrs(e: &BytesStart) -> Result<MjcfCompiler> {
    let a = Attrs::new(e)?;
    let mut compiler = MjcfCompiler::default();

    // A1. angle — angular unit
    if let Some(angle) = a.text("angle") {
        match angle.to_lowercase().as_str() {
            "degree" => compiler.angle = AngleUnit::Degree,
            "radian" => compiler.angle = AngleUnit::Radian,
            other => {
                return Err(MjcfError::invalid_keyword("compiler", "angle", other));
            }
        }
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

    // A3. meshdir / texturedir / assetdir — asset path resolution
    if let Some(meshdir) = a.string("meshdir") {
        compiler.meshdir = Some(meshdir);
    }
    if let Some(texturedir) = a.string("texturedir") {
        compiler.texturedir = Some(texturedir);
    }
    if let Some(assetdir) = a.string("assetdir") {
        compiler.assetdir = Some(assetdir);
    }

    // A4. autolimits
    if let Some(autolimits) = a.text("autolimits") {
        compiler.autolimits = autolimits == "true";
    }

    // A5. inertiafromgeom
    if let Some(ifg) = a.text("inertiafromgeom") {
        match ifg.to_lowercase().as_str() {
            "false" => compiler.inertiafromgeom = InertiaFromGeom::False,
            "true" => compiler.inertiafromgeom = InertiaFromGeom::True,
            "auto" => compiler.inertiafromgeom = InertiaFromGeom::Auto,
            other => {
                return Err(MjcfError::invalid_keyword(
                    "compiler",
                    "inertiafromgeom",
                    other,
                ));
            }
        }
    }

    // A6. boundmass / boundinertia
    if let Some(bm) = a.real("boundmass")? {
        compiler.boundmass = bm;
    }
    if let Some(bi) = a.real("boundinertia")? {
        compiler.boundinertia = bi;
    }

    // A7. balanceinertia
    if let Some(balance) = a.text("balanceinertia") {
        compiler.balanceinertia = balance == "true";
    }

    // A8. settotalmass
    if let Some(stm) = a.real("settotalmass")? {
        compiler.settotalmass = stm;
    }

    // A9. strippath
    if let Some(sp) = a.text("strippath") {
        compiler.strippath = sp == "true";
    }

    // A10. discardvisual
    if let Some(dv) = a.text("discardvisual") {
        compiler.discardvisual = dv == "true";
    }

    // A11. fusestatic
    if let Some(fs) = a.text("fusestatic") {
        compiler.fusestatic = fs == "true";
    }

    // A12. coordinate (deprecated — reject "global")
    if let Some(coord) = a.text("coordinate") {
        match coord.to_lowercase().as_str() {
            "local" => {} // no-op, matches current behavior
            "global" => {
                return Err(MjcfError::invalid_value(
                    "compiler",
                    "coordinate",
                    "coordinate='global' was removed in MuJoCo 2.3.4",
                ));
            }
            other => {
                return Err(MjcfError::invalid_keyword("compiler", "coordinate", other));
            }
        }
    }

    // Deferred attributes (stored but no behavior yet)
    if let Some(fitaabb) = a.text("fitaabb") {
        compiler.fitaabb = fitaabb == "true";
    }
    if let Some(usethread) = a.text("usethread") {
        compiler.usethread = usethread == "true";
    }
    if let Some(alignfree) = a.text("alignfree") {
        compiler.alignfree = alignfree == "true";
    }

    // A13. exactmeshinertia (deprecated — no behavioral effect since MuJoCo 3.5.0)
    if let Some(emi) = a.text("exactmeshinertia") {
        compiler.exactmeshinertia = emi == "true";
        tracing::warn!(
            "exactmeshinertia is deprecated and has no effect; \
             use <mesh inertia=\"exact\"/> instead"
        );
    }

    Ok(compiler)
}

/// Parse flag element attributes.
pub(super) fn parse_flag_attrs(e: &BytesStart) -> Result<MjcfFlag> {
    let a = Attrs::new(e)?;
    let mut flag = MjcfFlag::default();

    // Helper to parse enable/disable attributes
    let parse_flag = |name: &str, default: bool| a.text(name).map_or(default, |v| v != "disable");

    // Disable flags (true = enabled, false = disabled)
    flag.constraint = parse_flag("constraint", flag.constraint);
    flag.equality = parse_flag("equality", flag.equality);
    flag.frictionloss = parse_flag("frictionloss", flag.frictionloss);
    flag.limit = parse_flag("limit", flag.limit);
    flag.contact = parse_flag("contact", flag.contact);
    // MuJoCo 3.3.6+ split `passive` into independent `spring` + `damper`.
    // `passive` is silently ignored (unrecognized attributes are skipped).
    flag.spring = parse_flag("spring", flag.spring);
    flag.damper = parse_flag("damper", flag.damper);
    flag.gravity = parse_flag("gravity", flag.gravity);
    flag.clampctrl = parse_flag("clampctrl", flag.clampctrl);
    flag.warmstart = parse_flag("warmstart", flag.warmstart);
    flag.filterparent = parse_flag("filterparent", flag.filterparent);
    flag.actuation = parse_flag("actuation", flag.actuation);
    flag.refsafe = parse_flag("refsafe", flag.refsafe);
    flag.sensor = parse_flag("sensor", flag.sensor);
    flag.midphase = parse_flag("midphase", flag.midphase);
    flag.nativeccd = parse_flag("nativeccd", flag.nativeccd);
    flag.eulerdamp = parse_flag("eulerdamp", flag.eulerdamp);
    flag.autoreset = parse_flag("autoreset", flag.autoreset);
    flag.island = parse_flag("island", flag.island);
    // Enable flags (false = disabled, true = enabled)
    flag.override_contacts = parse_flag("override", flag.override_contacts);
    flag.energy = parse_flag("energy", flag.energy);
    flag.fwdinv = parse_flag("fwdinv", flag.fwdinv);
    flag.invdiscrete = parse_flag("invdiscrete", flag.invdiscrete);
    flag.multiccd = parse_flag("multiccd", flag.multiccd);
    flag.sleep = parse_flag("sleep", flag.sleep);

    Ok(flag)
}
