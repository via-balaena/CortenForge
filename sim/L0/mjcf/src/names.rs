//! A repeated name, refused as MuJoCo 3.5.0 refuses it (book L23).
//!
//! MuJoCo checks a kind's names whenever its reader names an element of it
//! (`mjs_setName`, `user_api.cc:1734-1749`), and again when it compiles the
//! model (`ProcessLists`, `user_model.cc:4888`): of one kind, two elements
//! with one non-empty name are refused, "repeated name 'n' in `kind`"
//! (`CheckRepeat`, `user_model.cc:4448-4475`). The world body is named
//! `world`. Elements of different kinds may share a name. Its reader takes
//! the sections in its own order, the world's bodies last
//! (`mjXReader::Parse`, `xml_native_reader.cc:990-1080`).
//!
//! [`check_names`] takes the kinds in that order, on the model with its
//! classes applied and its frames and composites expanded, where this crate
//! checked a body's, a joint's and an actuator's name before; so a name a
//! composite makes is checked where the composite stood. A frame's name,
//! which expanding the frames drops, is checked by [`check_frame_names`]
//! before they are expanded. A fault MuJoCo's reader meets before a
//! repeated name (an undefined class, a composite's) is refused here first
//! wherever this crate meets it first (ledger L353).
//!
//! This crate does not read a camera, light, material, texture or `<custom>`
//! element, so their names are not checked (ledger L376). A mesh's and a
//! height field's are checked by `builder::mesh::name_assets`, which names
//! the unnamed ones by their files first.

use std::collections::{HashMap, HashSet};

use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{EqualityKind, MjcfBody, MjcfFrame, MjcfModel};

/// The world body's name (`mjCModel`'s first body).
const WORLD: &str = "world";

/// The names met so far, one set per kind.
#[derive(Default)]
struct Seen(HashMap<ElementKind, HashSet<String>>);

impl Seen {
    /// Refuses `name` if an earlier element of `kind` has it; an empty name
    /// is no name. `at` locates the element.
    fn add(&mut self, kind: ElementKind, name: &str, at: impl FnOnce() -> Location) -> Result<()> {
        if name.is_empty() || self.0.entry(kind).or_default().insert(name.to_owned()) {
            Ok(())
        } else {
            Err(MjcfError::duplicate_name(kind, name).at(at()))
        }
    }
}

/// Refuses the first element whose name an earlier one of its kind has, in
/// MuJoCo 3.5.0's reading order: the contact pairs, then exclusions; the
/// flexes, then skins; the equalities in document order; the tendons,
/// actuators, sensors and keyframes; then the world's tree, each body
/// before its joints, geoms and sites, then its child bodies; then the
/// extension's plugin instances, which MuJoCo checks only when it compiles.
///
/// Within a section and within a body this crate keeps each kind apart, so
/// of two repeats there, the one it names can be another than MuJoCo's
/// (ledger L353).
pub fn check_names(model: &MjcfModel) -> Result<()> {
    let mut seen = Seen::default();
    for pair in &model.contact.pairs {
        seen.add(
            ElementKind::Pair,
            pair.name.as_deref().unwrap_or(""),
            Location::unknown,
        )?;
    }
    for exclude in &model.contact.excludes {
        let name = exclude.name.as_deref().unwrap_or("");
        seen.add(ElementKind::Exclude, name, Location::unknown)?;
    }
    for flex in &model.flex {
        seen.add(ElementKind::Flex, &flex.name, Location::unknown)?;
    }
    for skin in &model.skins {
        seen.add(ElementKind::Skin, &skin.name, Location::unknown)?;
    }
    let equality = &model.equality;
    for &(kind, idx) in &equality.order {
        let name = match kind {
            EqualityKind::Connect => equality.connects[idx].name.as_deref(),
            EqualityKind::Weld => equality.welds[idx].name.as_deref(),
            EqualityKind::Joint => equality.joints[idx].name.as_deref(),
            EqualityKind::Distance => equality.distances[idx].name.as_deref(),
            EqualityKind::Tendon => equality.tendons[idx].name.as_deref(),
        };
        seen.add(ElementKind::Equality, name.unwrap_or(""), Location::unknown)?;
    }
    for tendon in &model.tendons {
        seen.add(ElementKind::Tendon, &tendon.name, Location::unknown)?;
    }
    for actuator in &model.actuators {
        seen.add(ElementKind::Actuator, &actuator.name, Location::unknown)?;
    }
    for sensor in &model.sensors {
        seen.add(ElementKind::Sensor, &sensor.name, Location::unknown)?;
    }
    for key in &model.keyframes {
        seen.add(ElementKind::Keyframe, &key.name, Location::unknown)?;
    }
    seen.add(ElementKind::Body, WORLD, Location::unknown)?;
    // The tree, without recursing (it is as deep as `check_tree_depth`
    // lets it be): a body, then its elements, then its children in order.
    body_elements(&model.worldbody, &mut seen)?;
    let mut stack: Vec<&MjcfBody> = model.worldbody.children.iter().rev().collect();
    while let Some(body) = stack.pop() {
        seen.add(ElementKind::Body, &body.name, Location::unknown)?;
        body_elements(body, &mut seen)?;
        stack.extend(body.children.iter().rev());
    }
    for plugin in model.extensions.iter().flat_map(|e| &e.plugins) {
        for instance in &plugin.instances {
            seen.add(ElementKind::Plugin, &instance.name, Location::unknown)?;
        }
    }
    Ok(())
}

/// A body's joints', geoms' and sites' names, for [`check_names`].
fn body_elements(body: &MjcfBody, seen: &mut Seen) -> Result<()> {
    let at = |kind, name: &str| Location::named(kind, name).in_body(&body.name);
    for joint in &body.joints {
        seen.add(ElementKind::Joint, &joint.name, || {
            at(ElementKind::Joint, &joint.name)
        })?;
    }
    for geom in &body.geoms {
        let name = geom.name.as_deref().unwrap_or("");
        seen.add(ElementKind::Geom, name, || at(ElementKind::Geom, name))?;
    }
    for site in &body.sites {
        seen.add(ElementKind::Site, &site.name, || {
            at(ElementKind::Site, &site.name)
        })?;
    }
    Ok(())
}

/// Refuses the first `<frame>` whose name an earlier frame has, on the tree
/// before its frames are expanded, read in document order as far as this
/// crate keeps it: a body's frames before its child bodies, a frame's
/// nested frames before its bodies.
pub fn check_frame_names(worldbody: &MjcfBody) -> Result<()> {
    enum Next<'a> {
        Body(&'a MjcfBody),
        Frame(&'a MjcfFrame, &'a str),
    }
    let mut seen = Seen::default();
    let mut stack = vec![Next::Body(worldbody)];
    while let Some(next) = stack.pop() {
        let pending: Vec<Next<'_>> = match next {
            Next::Body(body) => {
                let frames = body.frames.iter().map(|f| Next::Frame(f, &body.name));
                frames.chain(body.children.iter().map(Next::Body)).collect()
            }
            Next::Frame(frame, body) => {
                let name = frame.name.as_deref().unwrap_or("");
                seen.add(ElementKind::Frame, name, || {
                    Location::named(ElementKind::Frame, name).in_body(body)
                })?;
                let frames = frame.frames.iter().map(|f| Next::Frame(f, body));
                frames.chain(frame.bodies.iter().map(Next::Body)).collect()
            }
        };
        stack.extend(pending.into_iter().rev());
    }
    Ok(())
}
