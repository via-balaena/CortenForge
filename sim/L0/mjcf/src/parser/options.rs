//! `<option>`, `<compiler>`, `<size>`, and `<flag>` parsing.

use quick_xml::Reader;
use quick_xml::events::{BytesStart, Event};
use std::io::BufRead;

use crate::error::{MjcfError, Result};

use crate::types::{
    AngleUnit, InertiaFromGeom, MjcfCompiler, MjcfConeType, MjcfFlag, MjcfIntegrator,
    MjcfJacobianType, MjcfModel, MjcfOption, MjcfSolverType,
};

use super::attrs::{
    attr_fixed, get_attribute_opt, parse_float_attr, parse_int_attr, parse_vector3,
};

/// Parse `<size>` element attributes into MjcfModel nuser_* fields.
/// Multiple `<size>` elements merge with last-writer-wins per attribute.
pub(super) fn parse_size_attrs(e: &BytesStart, model: &mut MjcfModel) {
    if let Some(val) = parse_int_attr(e, "nuser_body") {
        model.nuser_body = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_jnt") {
        model.nuser_jnt = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_geom") {
        model.nuser_geom = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_site") {
        model.nuser_site = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_tendon") {
        model.nuser_tendon = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_actuator") {
        model.nuser_actuator = val;
    }
    if let Some(val) = parse_int_attr(e, "nuser_sensor") {
        model.nuser_sensor = val;
    }
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
            Ok(Event::Eof) => return Err(MjcfError::XmlParse("unexpected EOF in option".into())),
            Ok(_) => {}
            Err(e) => return Err(MjcfError::XmlParse(e.to_string())),
        }
        buf.clear();
    }

    Ok(option)
}

/// Parse option attributes only.
pub(super) fn parse_option_attrs(e: &BytesStart) -> Result<MjcfOption> {
    let mut option = MjcfOption::default();

    // Core simulation
    if let Some(ts) = parse_float_attr(e, "timestep") {
        option.timestep = ts;
    }
    if let Some(integrator) = get_attribute_opt(e, "integrator") {
        if let Some(int) = MjcfIntegrator::from_str(&integrator) {
            option.integrator = int;
        }
    }

    // Solver configuration
    if let Some(solver) = get_attribute_opt(e, "solver") {
        if let Some(s) = MjcfSolverType::from_str(&solver) {
            option.solver = s;
        }
    }
    if let Some(iter) = parse_int_attr(e, "iterations") {
        option.iterations = iter.max(0) as usize;
    }
    if let Some(tol) = parse_float_attr(e, "tolerance") {
        option.tolerance = tol;
    }
    if let Some(ls_iter) = parse_int_attr(e, "ls_iterations") {
        option.ls_iterations = ls_iter.max(0) as usize;
    }
    if let Some(ls_tol) = parse_float_attr(e, "ls_tolerance") {
        option.ls_tolerance = ls_tol;
    }
    if let Some(noslip) = parse_int_attr(e, "noslip_iterations") {
        option.noslip_iterations = noslip.max(0) as usize;
    }
    if let Some(noslip_tol) = parse_float_attr(e, "noslip_tolerance") {
        option.noslip_tolerance = noslip_tol;
    }
    if let Some(ccd) = parse_int_attr(e, "ccd_iterations") {
        option.ccd_iterations = ccd.max(0) as usize;
    }
    if let Some(ccd_tol) = parse_float_attr(e, "ccd_tolerance") {
        option.ccd_tolerance = ccd_tol;
    }
    if let Some(sdf_iter) = parse_int_attr(e, "sdf_iterations") {
        option.sdf_iterations = sdf_iter.max(0) as usize;
    }
    if let Some(sdf_init) = parse_int_attr(e, "sdf_initpoints") {
        option.sdf_initpoints = sdf_init.max(0) as usize;
    }

    // Contact configuration
    if let Some(cone) = get_attribute_opt(e, "cone") {
        if let Some(c) = MjcfConeType::from_str(&cone) {
            option.cone = c;
        }
    }
    if let Some(jacobian) = get_attribute_opt(e, "jacobian") {
        if let Some(j) = MjcfJacobianType::from_str(&jacobian) {
            option.jacobian = j;
        }
    }
    if let Some(impratio) = parse_float_attr(e, "impratio") {
        option.impratio = impratio;
    }
    if let Some(regularization) = parse_float_attr(e, "regularization") {
        option.regularization = regularization;
    }
    if let Some(fsmooth) = parse_float_attr(e, "friction_smoothing") {
        option.friction_smoothing = fsmooth;
    }

    // Physics environment
    if let Some(grav) = get_attribute_opt(e, "gravity") {
        option.gravity = parse_vector3(&grav)?;
    }
    if let Some(wind) = get_attribute_opt(e, "wind") {
        option.wind = parse_vector3(&wind)?;
    }
    if let Some(magnetic) = get_attribute_opt(e, "magnetic") {
        option.magnetic = parse_vector3(&magnetic)?;
    }
    if let Some(density) = parse_float_attr(e, "density") {
        option.density = density;
    }
    if let Some(viscosity) = parse_float_attr(e, "viscosity") {
        option.viscosity = viscosity;
    }

    // Constraint limits
    if let Some(nconmax) = parse_int_attr(e, "nconmax") {
        option.nconmax = nconmax.max(0) as usize;
    }
    if let Some(njmax) = parse_int_attr(e, "njmax") {
        option.njmax = njmax.max(0) as usize;
    }

    // Overrides
    if let Some(o_margin) = parse_float_attr(e, "o_margin") {
        option.o_margin = o_margin;
    }
    option.o_solimp = attr_fixed::<5>(e, "o_solimp")?;
    option.o_solref = attr_fixed::<2>(e, "o_solref")?;
    option.o_friction = attr_fixed::<5>(e, "o_friction")?;

    // Sleep
    if let Some(st) = parse_float_attr(e, "sleep_tolerance") {
        option.sleep_tolerance = st;
    }

    // Per-group actuator disabling
    if let Some(groups_str) = get_attribute_opt(e, "actuatorgroupdisable") {
        for token in groups_str.split_whitespace() {
            if let Ok(group) = token.parse::<i32>() {
                if (0..=30).contains(&group) {
                    // `group` is bound to 0..=30 by the contains check above.
                    #[allow(clippy::cast_sign_loss)]
                    {
                        option.actuatorgroupdisable |= 1u32 << (group as u32);
                    }
                }
            }
        }
    }

    Ok(option)
}

/// Parse compiler element attributes.
pub(super) fn parse_compiler_attrs(e: &BytesStart) -> Result<MjcfCompiler> {
    let mut compiler = MjcfCompiler::default();

    // A1. angle — angular unit
    if let Some(angle) = get_attribute_opt(e, "angle") {
        match angle.to_lowercase().as_str() {
            "degree" => compiler.angle = AngleUnit::Degree,
            "radian" => compiler.angle = AngleUnit::Radian,
            other => {
                return Err(MjcfError::invalid_attribute(
                    "angle",
                    "compiler",
                    format!("expected 'degree' or 'radian', got '{other}'"),
                ));
            }
        }
    }

    // A2. eulerseq — Euler rotation sequence
    if let Some(seq) = get_attribute_opt(e, "eulerseq") {
        if seq.len() != 3
            || !seq
                .chars()
                .all(|c| matches!(c, 'x' | 'y' | 'z' | 'X' | 'Y' | 'Z'))
        {
            return Err(MjcfError::invalid_attribute(
                "eulerseq",
                "compiler",
                format!("expected 3 characters from {{x,y,z,X,Y,Z}}, got '{seq}'"),
            ));
        }
        compiler.eulerseq = seq;
    }

    // A3. meshdir / texturedir / assetdir — asset path resolution
    if let Some(meshdir) = get_attribute_opt(e, "meshdir") {
        compiler.meshdir = Some(meshdir);
    }
    if let Some(texturedir) = get_attribute_opt(e, "texturedir") {
        compiler.texturedir = Some(texturedir);
    }
    if let Some(assetdir) = get_attribute_opt(e, "assetdir") {
        compiler.assetdir = Some(assetdir);
    }

    // A4. autolimits
    if let Some(autolimits) = get_attribute_opt(e, "autolimits") {
        compiler.autolimits = autolimits == "true";
    }

    // A5. inertiafromgeom
    if let Some(ifg) = get_attribute_opt(e, "inertiafromgeom") {
        match ifg.to_lowercase().as_str() {
            "false" => compiler.inertiafromgeom = InertiaFromGeom::False,
            "true" => compiler.inertiafromgeom = InertiaFromGeom::True,
            "auto" => compiler.inertiafromgeom = InertiaFromGeom::Auto,
            other => {
                return Err(MjcfError::invalid_attribute(
                    "inertiafromgeom",
                    "compiler",
                    format!("expected 'false', 'true', or 'auto', got '{other}'"),
                ));
            }
        }
    }

    // A6. boundmass / boundinertia
    if let Some(bm) = parse_float_attr(e, "boundmass") {
        compiler.boundmass = bm;
    }
    if let Some(bi) = parse_float_attr(e, "boundinertia") {
        compiler.boundinertia = bi;
    }

    // A7. balanceinertia
    if let Some(balance) = get_attribute_opt(e, "balanceinertia") {
        compiler.balanceinertia = balance == "true";
    }

    // A8. settotalmass
    if let Some(stm) = parse_float_attr(e, "settotalmass") {
        compiler.settotalmass = stm;
    }

    // A9. strippath
    if let Some(sp) = get_attribute_opt(e, "strippath") {
        compiler.strippath = sp == "true";
    }

    // A10. discardvisual
    if let Some(dv) = get_attribute_opt(e, "discardvisual") {
        compiler.discardvisual = dv == "true";
    }

    // A11. fusestatic
    if let Some(fs) = get_attribute_opt(e, "fusestatic") {
        compiler.fusestatic = fs == "true";
    }

    // A12. coordinate (deprecated — reject "global")
    if let Some(coord) = get_attribute_opt(e, "coordinate") {
        match coord.to_lowercase().as_str() {
            "local" => {} // no-op, matches current behavior
            "global" => {
                return Err(MjcfError::Unsupported(
                    "coordinate='global' was removed in MuJoCo 2.3.4".to_string(),
                ));
            }
            other => {
                return Err(MjcfError::invalid_attribute(
                    "coordinate",
                    "compiler",
                    format!("expected 'local' or 'global', got '{other}'"),
                ));
            }
        }
    }

    // Deferred attributes (stored but no behavior yet)
    if let Some(fitaabb) = get_attribute_opt(e, "fitaabb") {
        compiler.fitaabb = fitaabb == "true";
    }
    if let Some(usethread) = get_attribute_opt(e, "usethread") {
        compiler.usethread = usethread == "true";
    }
    if let Some(alignfree) = get_attribute_opt(e, "alignfree") {
        compiler.alignfree = alignfree == "true";
    }

    // A13. exactmeshinertia (deprecated — no behavioral effect since MuJoCo 3.5.0)
    if let Some(emi) = get_attribute_opt(e, "exactmeshinertia") {
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
    let mut flag = MjcfFlag::default();

    // Helper to parse enable/disable attributes
    fn parse_flag(e: &BytesStart, name: &str, default: bool) -> bool {
        get_attribute_opt(e, name).map_or(default, |v| v != "disable")
    }

    // Disable flags (true = enabled, false = disabled)
    flag.constraint = parse_flag(e, "constraint", flag.constraint);
    flag.equality = parse_flag(e, "equality", flag.equality);
    flag.frictionloss = parse_flag(e, "frictionloss", flag.frictionloss);
    flag.limit = parse_flag(e, "limit", flag.limit);
    flag.contact = parse_flag(e, "contact", flag.contact);
    // MuJoCo 3.3.6+ split `passive` into independent `spring` + `damper`.
    // `passive` is silently ignored (unrecognized attributes are skipped).
    flag.spring = parse_flag(e, "spring", flag.spring);
    flag.damper = parse_flag(e, "damper", flag.damper);
    flag.gravity = parse_flag(e, "gravity", flag.gravity);
    flag.clampctrl = parse_flag(e, "clampctrl", flag.clampctrl);
    flag.warmstart = parse_flag(e, "warmstart", flag.warmstart);
    flag.filterparent = parse_flag(e, "filterparent", flag.filterparent);
    flag.actuation = parse_flag(e, "actuation", flag.actuation);
    flag.refsafe = parse_flag(e, "refsafe", flag.refsafe);
    flag.sensor = parse_flag(e, "sensor", flag.sensor);
    flag.midphase = parse_flag(e, "midphase", flag.midphase);
    flag.nativeccd = parse_flag(e, "nativeccd", flag.nativeccd);
    flag.eulerdamp = parse_flag(e, "eulerdamp", flag.eulerdamp);
    flag.autoreset = parse_flag(e, "autoreset", flag.autoreset);
    flag.island = parse_flag(e, "island", flag.island);
    // Enable flags (false = disabled, true = enabled)
    flag.override_contacts = parse_flag(e, "override", flag.override_contacts);
    flag.energy = parse_flag(e, "energy", flag.energy);
    flag.fwdinv = parse_flag(e, "fwdinv", flag.fwdinv);
    flag.invdiscrete = parse_flag(e, "invdiscrete", flag.invdiscrete);
    flag.multiccd = parse_flag(e, "multiccd", flag.multiccd);
    flag.sleep = parse_flag(e, "sleep", flag.sleep);

    Ok(flag)
}
