//! Every element's class applied in one pass over the parsed model, before
//! frames, composites and the compiler passes move or make elements.
//!
//! MuJoCo 3.5.0 applies an element's class when it reads the element
//! (`mjXReader::GetClass`, then the element's own attributes over the
//! class's), so a frame composes, and fusion moves, the element its class
//! made, and the elements a `<composite>` makes later take none of the
//! user's classes (`mjCComposite` builds them from its own default). The
//! builder reads each element as this pass leaves it.

use super::geom::{resolve_geom_fromto, resolve_site_fromto};
use crate::defaults::DefaultResolver;
use crate::error::{ElementKind, Location, Result};
use crate::types::{MjcfBody, MjcfFrame, MjcfGeom, MjcfModel, MjcfSite};

/// A body or frame still to visit, with the `childclass` its contents take
/// where they name no class of their own, and (for a frame) the body that
/// holds it.
enum Pending<'a> {
    Body(&'a mut MjcfBody, Option<String>),
    Frame(&'a mut MjcfFrame, Option<String>, String),
}

/// Each geom, joint and site of the body tree given its class: its own, else
/// the `childclass` of the nearest body or frame holding it (`main` where
/// none has one). A geom's and a site's `fromto` is then made into its `pos`,
/// `quat` and `size` in its own frame, which a frame or fusion composes
/// after, as MuJoCo compiles the element before either. Each `childclass` is
/// checked where it is written. The walk keeps its own stack, so a deep tree
/// costs no call depth.
///
/// # Errors
///
/// [`MjcfError::UnknownClass`](crate::MjcfError::UnknownClass) for a `class`
/// or `childclass` no `<default>` makes; a `fromto` that
/// `resolve_geom_fromto` or `resolve_site_fromto` refuses.
pub fn apply_defaults(mjcf: &mut MjcfModel, resolver: &DefaultResolver) -> Result<()> {
    let mut pending = vec![Pending::Body(&mut mjcf.worldbody, None)];
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
                for joint in joints.iter_mut() {
                    if joint.class.is_none() {
                        joint.class.clone_from(&effective);
                    }
                    let at = Location::named(ElementKind::Joint, &joint.name).in_body(name);
                    *joint = resolver.apply_to_joint(joint).map_err(|e| e.at(at))?;
                }
                apply_to_geoms(geoms, resolver, effective.as_deref(), name)?;
                apply_to_sites(sites, resolver, effective.as_deref(), name)?;
                // Popped in document order: frames, then bodies, each as written.
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
