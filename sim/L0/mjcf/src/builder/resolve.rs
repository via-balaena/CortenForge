//! Every element's class applied in one pass over the parsed model, before
//! frames, composites and the compiler passes move or make elements.
//!
//! MuJoCo 3.5.0 applies an element's class when it reads the element
//! (`mjXReader::GetClass`, then the element's own attributes over the
//! class's), so a frame composes, and fusion moves, the element its class
//! made, and the elements a `<composite>` makes later take none of the
//! user's classes (`mjCComposite` builds them from its own default). The
//! builder reads each element as this pass leaves it and applies no class.

use super::actuator::{check_muscle_values, check_servo_gains};
use super::geom::{resolve_geom_fromto, resolve_site_fromto};
use crate::defaults::DefaultResolver;
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{
    EqualityKind, MjcfBody, MjcfEquality, MjcfFrame, MjcfGeom, MjcfModel, MjcfSite,
};

/// A body or frame still to visit, with the `childclass` its contents take
/// where they name no class of their own, and (for a frame) the body that
/// holds it.
enum Pending<'a> {
    Body(&'a mut MjcfBody, Option<String>),
    Frame(&'a mut MjcfFrame, Option<String>, String),
}

/// Every element given its class: the body tree's (`apply_to_body_tree`),
/// then each mesh, actuator, tendon, sensor, contact pair and equality. An
/// actuator's own servo gains and muscle values are checked before its class
/// gives any.
///
/// # Errors
///
/// [`MjcfError::UnknownClass`](crate::MjcfError::UnknownClass) for a `class`
/// or `childclass` no `<default>` makes; an actuator's own servo gains or
/// muscle values that `check_servo_gains` or `check_muscle_values` refuses,
/// or a muscle force `apply_to_actuator` refuses; a `fromto` the body tree's
/// pass refuses;
/// [`MjcfError::InvalidModel`](crate::MjcfError::InvalidModel) for an
/// equality order naming an equality the model does not hold.
pub fn apply_defaults(mjcf: &mut MjcfModel, resolver: &DefaultResolver) -> Result<()> {
    apply_to_body_tree(&mut mjcf.worldbody, resolver)?;
    for mesh in &mut mjcf.meshes {
        let at = Location::named(ElementKind::Mesh, &mesh.name);
        *mesh = resolver.apply_to_mesh(mesh).map_err(|e| e.at(at))?;
    }
    for (i, actuator) in mjcf.actuators.iter_mut().enumerate() {
        let at = Location::nth(ElementKind::Actuator, Some(&actuator.name), i);
        // A servo's own gains and a muscle's own values, before its class
        // gives any (the parser checked a parsed one; this holds an actuator
        // built in code to it).
        let element = actuator.actuator_type.as_str();
        let own = check_servo_gains(element, actuator.kv, actuator.dampratio, actuator.timeconst)
            .and_then(|()| {
                check_muscle_values(
                    element,
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
                )
            });
        *actuator = resolver
            .apply_to_actuator(actuator)
            .and_then(|resolved| own.map(|()| resolved))
            .map_err(|e| e.with_location_if_unknown(at))?;
    }
    for (i, tendon) in mjcf.tendons.iter_mut().enumerate() {
        let at = Location::nth(ElementKind::Tendon, Some(&tendon.name), i);
        *tendon = resolver.apply_to_tendon(tendon).map_err(|e| e.at(at))?;
    }
    for (i, sensor) in mjcf.sensors.iter_mut().enumerate() {
        let at = Location::nth(ElementKind::Sensor, Some(&sensor.name), i);
        *sensor = resolver.apply_to_sensor(sensor).map_err(|e| e.at(at))?;
    }
    for (i, pair) in mjcf.contact.pairs.iter_mut().enumerate() {
        let at = Location::nth(ElementKind::Pair, pair.name.as_deref(), i);
        *pair = resolver.apply_to_pair(pair).map_err(|e| e.at(at))?;
    }
    apply_to_equalities(&mut mjcf.equality, resolver)
}

/// Each equality given its class, numbered among them all in document order.
///
/// # Errors
///
/// [`MjcfError::UnknownClass`](crate::MjcfError::UnknownClass) for a class no
/// `<default>` makes; [`MjcfError::InvalidModel`](crate::MjcfError::InvalidModel)
/// where the order names an equality its lists do not hold (a model built in
/// code).
fn apply_to_equalities(equality: &mut MjcfEquality, resolver: &DefaultResolver) -> Result<()> {
    let MjcfEquality {
        connects,
        welds,
        joints,
        distances,
        tendons,
        order,
        ..
    } = equality;
    for (i, &(kind, k)) in order.iter().enumerate() {
        // The equality's name, class, and the three values a class gives.
        let (name, class, active, solref, solimp) = match kind {
            EqualityKind::Connect => connects.get_mut(k).map(|e| {
                (
                    &e.name,
                    &e.class,
                    &mut e.active,
                    &mut e.solref,
                    &mut e.solimp,
                )
            }),
            EqualityKind::Weld => welds.get_mut(k).map(|e| {
                (
                    &e.name,
                    &e.class,
                    &mut e.active,
                    &mut e.solref,
                    &mut e.solimp,
                )
            }),
            EqualityKind::Joint => joints.get_mut(k).map(|e| {
                (
                    &e.name,
                    &e.class,
                    &mut e.active,
                    &mut e.solref,
                    &mut e.solimp,
                )
            }),
            EqualityKind::Distance => distances.get_mut(k).map(|e| {
                (
                    &e.name,
                    &e.class,
                    &mut e.active,
                    &mut e.solref,
                    &mut e.solimp,
                )
            }),
            EqualityKind::Tendon => tendons.get_mut(k).map(|e| {
                (
                    &e.name,
                    &e.class,
                    &mut e.active,
                    &mut e.solref,
                    &mut e.solimp,
                )
            }),
        }
        .ok_or_else(|| {
            MjcfError::invalid_model(format!(
                "the equality order names {} #{k}, which the model does not hold",
                equality_tag(kind)
            ))
            .at(Location::nth(ElementKind::Equality, None, i))
        })?;
        resolver
            .apply_equality_defaults(class.as_deref(), active, solref, solimp)
            .map_err(|e| e.at(Location::nth(ElementKind::Equality, name.as_deref(), i)))?;
    }
    Ok(())
}

/// An equality kind's MJCF tag.
fn equality_tag(kind: EqualityKind) -> &'static str {
    match kind {
        EqualityKind::Connect => "connect",
        EqualityKind::Weld => "weld",
        EqualityKind::Joint => "joint",
        EqualityKind::Distance => "distance",
        EqualityKind::Tendon => "tendon",
    }
}

/// Each geom, joint and site of the body tree given its class: its own, else
/// the `childclass` of the nearest body or frame holding it (`main` where
/// none has one); a `<freejoint>` takes none, as MuJoCo makes it. A geom's
/// and a site's `fromto` is then made into its `pos`, `quat` and `size` in
/// its own frame, which a frame or fusion composes after, as MuJoCo compiles
/// the element before either. Each `childclass` is
/// checked where it is written. The walk keeps its own stack, so a deep tree
/// costs no call depth.
fn apply_to_body_tree(world: &mut MjcfBody, resolver: &DefaultResolver) -> Result<()> {
    let mut pending = vec![Pending::Body(world, None)];
    while let Some(next) = pending.pop() {
        match next {
            Pending::Body(body, inherited) => {
                let MjcfBody {
                    name,
                    childclass,
                    joints,
                    geoms,
                    sites,
                    frames,
                    children,
                    ..
                } = body;
                if let Some(cc) = childclass.as_deref() {
                    resolver
                        .class(Some(cc))
                        .map_err(|e| e.at(Location::named(ElementKind::Body, name)))?;
                }
                let effective = childclass.clone().or(inherited);
                // A `<freejoint>` takes no class.
                for joint in joints.iter_mut().filter(|j| !j.freejoint) {
                    if joint.class.is_none() {
                        joint.class.clone_from(&effective);
                    }
                    let at = Location::named(ElementKind::Joint, &joint.name).in_body(name);
                    *joint = resolver.apply_to_joint(joint).map_err(|e| e.at(at))?;
                }
                apply_to_geoms(geoms, resolver, effective.as_deref(), name)?;
                apply_to_sites(sites, resolver, effective.as_deref(), name)?;
                // Popped frames first, then child bodies, each in the order
                // written; a body's elements are visited by kind (joints,
                // geoms, sites), not as they interleave in the document.
                for child in children.iter_mut().rev() {
                    pending.push(Pending::Body(child, effective.clone()));
                }
                for frame in frames.iter_mut().rev() {
                    pending.push(Pending::Frame(frame, effective.clone(), name.clone()));
                }
            }
            Pending::Frame(frame, inherited, body) => {
                if let Some(cc) = frame.childclass.as_deref() {
                    resolver.class(Some(cc)).map_err(|e| {
                        e.at(Location::named(
                            ElementKind::Frame,
                            frame.name.as_deref().unwrap_or(""),
                        )
                        .in_body(&body))
                    })?;
                }
                let effective = frame.childclass.clone().or(inherited);
                apply_to_geoms(&mut frame.geoms, resolver, effective.as_deref(), &body)?;
                apply_to_sites(&mut frame.sites, resolver, effective.as_deref(), &body)?;
                for child in frame.bodies.iter_mut().rev() {
                    pending.push(Pending::Body(child, effective.clone()));
                }
                for sub in frame.frames.iter_mut().rev() {
                    pending.push(Pending::Frame(sub, effective.clone(), body.clone()));
                }
            }
        }
    }
    Ok(())
}

/// Each geom of `body` (directly or in one of its frames) given its class,
/// then its `fromto`.
fn apply_to_geoms(
    geoms: &mut [MjcfGeom],
    resolver: &DefaultResolver,
    childclass: Option<&str>,
    body: &str,
) -> Result<()> {
    for geom in geoms {
        if geom.class.is_none() {
            geom.class = childclass.map(str::to_string);
        }
        let at =
            Location::named(ElementKind::Geom, geom.name.as_deref().unwrap_or("")).in_body(body);
        *geom = resolver.apply_to_geom(geom).map_err(|e| e.at(at.clone()))?;
        resolve_geom_fromto(geom).map_err(|e| e.with_location_if_unknown(at))?;
    }
    Ok(())
}

/// Each site of `body` (directly or in one of its frames) given its class,
/// then its `fromto`.
fn apply_to_sites(
    sites: &mut [MjcfSite],
    resolver: &DefaultResolver,
    childclass: Option<&str>,
    body: &str,
) -> Result<()> {
    for site in sites {
        if site.class.is_none() {
            site.class = childclass.map(str::to_string);
        }
        let at = Location::named(ElementKind::Site, &site.name).in_body(body);
        *site = resolver.apply_to_site(site).map_err(|e| e.at(at.clone()))?;
        resolve_site_fromto(site).map_err(|e| e.with_location_if_unknown(at))?;
    }
    Ok(())
}
