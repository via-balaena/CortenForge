//! A body's or a frame's elements in the order the document writes them
//! ([`MjcfBody::items`], [`MjcfFrame::items`]), for the passes that expand
//! frames and composites where they are written.

use crate::types::{MjcfBody, MjcfComposite, MjcfFrame, MjcfGeom, MjcfItem, MjcfJoint, MjcfSite};

/// One element of a body or a frame, owned.
pub(super) enum Element {
    Joint(MjcfJoint),
    Geom(MjcfGeom),
    Site(MjcfSite),
    Body(MjcfBody),
    Frame(MjcfFrame),
    Composite(MjcfComposite),
}

/// What a body or a frame holds, moved out of it: each kind's list and the
/// `items` that order them.
#[derive(Default)]
pub(super) struct Contents {
    items: Vec<MjcfItem>,
    joints: Vec<MjcfJoint>,
    geoms: Vec<MjcfGeom>,
    sites: Vec<MjcfSite>,
    bodies: Vec<MjcfBody>,
    frames: Vec<MjcfFrame>,
    composites: Vec<MjcfComposite>,
}

impl Contents {
    /// A body's elements, moved out of it.
    pub(super) fn take_from_body(body: &mut MjcfBody) -> Self {
        Self {
            items: std::mem::take(&mut body.items),
            joints: std::mem::take(&mut body.joints),
            geoms: std::mem::take(&mut body.geoms),
            sites: std::mem::take(&mut body.sites),
            bodies: std::mem::take(&mut body.children),
            frames: std::mem::take(&mut body.frames),
            composites: std::mem::take(&mut body.composites),
        }
    }

    /// A frame's elements, moved out of it.
    pub(super) fn take_from_frame(frame: &mut MjcfFrame) -> Self {
        Self {
            items: std::mem::take(&mut frame.items),
            geoms: std::mem::take(&mut frame.geoms),
            sites: std::mem::take(&mut frame.sites),
            bodies: std::mem::take(&mut frame.bodies),
            frames: std::mem::take(&mut frame.frames),
            composites: std::mem::take(&mut frame.composites),
            ..Self::default()
        }
    }

    /// Its elements in the order `items` gives them, then each element no
    /// entry reaches, each list in its own order, frames and then composites
    /// last (`MjcfBody::items`).
    pub(super) fn into_elements(self) -> Vec<Element> {
        let Self {
            items,
            joints,
            geoms,
            sites,
            bodies,
            frames,
            composites,
        } = self;
        let mut joints = joints.into_iter();
        let mut geoms = geoms.into_iter();
        let mut sites = sites.into_iter();
        let mut bodies = bodies.into_iter();
        let mut frames = frames.into_iter();
        let mut composites = composites.into_iter();
        let mut elements: Vec<Element> = items
            .iter()
            .filter_map(|item| match item {
                MjcfItem::Joint => joints.next().map(Element::Joint),
                MjcfItem::Geom => geoms.next().map(Element::Geom),
                MjcfItem::Site => sites.next().map(Element::Site),
                MjcfItem::Body => bodies.next().map(Element::Body),
                MjcfItem::Frame => frames.next().map(Element::Frame),
                MjcfItem::Composite => composites.next().map(Element::Composite),
            })
            .collect();
        elements.extend(joints.map(Element::Joint));
        elements.extend(geoms.map(Element::Geom));
        elements.extend(sites.map(Element::Site));
        elements.extend(bodies.map(Element::Body));
        elements.extend(frames.map(Element::Frame));
        elements.extend(composites.map(Element::Composite));
        elements
    }

    /// `element` added after the elements already held.
    pub(super) fn push(&mut self, element: Element) {
        let item = match element {
            Element::Joint(joint) => {
                self.joints.push(joint);
                MjcfItem::Joint
            }
            Element::Geom(geom) => {
                self.geoms.push(geom);
                MjcfItem::Geom
            }
            Element::Site(site) => {
                self.sites.push(site);
                MjcfItem::Site
            }
            Element::Body(body) => {
                self.bodies.push(body);
                MjcfItem::Body
            }
            Element::Frame(frame) => {
                self.frames.push(frame);
                MjcfItem::Frame
            }
            Element::Composite(composite) => {
                self.composites.push(composite);
                MjcfItem::Composite
            }
        };
        self.items.push(item);
    }

    /// The elements put back into `body`, whose lists `take_from_body`
    /// emptied.
    pub(super) fn put_into_body(self, body: &mut MjcfBody) {
        body.items = self.items;
        body.joints = self.joints;
        body.geoms = self.geoms;
        body.sites = self.sites;
        body.children = self.bodies;
        body.frames = self.frames;
        body.composites = self.composites;
    }

    /// The elements put back into `frame`, whose lists `take_from_frame`
    /// emptied. A frame holds no joint, and none was taken from it.
    pub(super) fn put_into_frame(self, frame: &mut MjcfFrame) {
        debug_assert!(self.joints.is_empty(), "a frame holds no joint");
        frame.items = self.items;
        frame.geoms = self.geoms;
        frame.sites = self.sites;
        frame.bodies = self.bodies;
        frame.frames = self.frames;
        frame.composites = self.composites;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    /// A body holding one of each kind, written out of kind order.
    fn parsed_body() -> MjcfBody {
        let mut model = crate::parse_mjcf_str(
            r#"<mujoco><worldbody><body name="b"><composite type="cable" prefix="c" count="4 1 1" curve="s" size="1"><geom type="capsule" size=".01"/></composite><frame name="f"/><body name="c0"/><site name="s0"/><geom name="g0" size="1"/><joint name="j"/></body></worldbody></mujoco>"#,
        )
        .unwrap();
        model.worldbody.children.pop().unwrap()
    }

    fn names(elements: &[Element]) -> Vec<String> {
        elements
            .iter()
            .map(|e| match e {
                Element::Joint(j) => format!("joint {}", j.name),
                Element::Geom(g) => format!("geom {}", g.name.as_deref().unwrap_or("")),
                Element::Site(s) => format!("site {}", s.name),
                Element::Body(b) => format!("body {}", b.name),
                Element::Frame(f) => format!("frame {}", f.name.as_deref().unwrap_or("")),
                Element::Composite(c) => format!("composite {}", c.prefix),
            })
            .collect()
    }

    #[test]
    fn a_parsed_bodys_elements_come_in_the_documents_order() {
        let mut body = parsed_body();
        let elements = Contents::take_from_body(&mut body).into_elements();
        assert_eq!(
            names(&elements),
            [
                "composite c",
                "frame f",
                "body c0",
                "site s0",
                "geom g0",
                "joint j"
            ]
        );
    }

    #[test]
    fn with_no_items_each_lists_order_frames_then_composites_last() {
        let mut body = parsed_body();
        body.items.clear();
        let elements = Contents::take_from_body(&mut body).into_elements();
        assert_eq!(
            names(&elements),
            [
                "joint j",
                "geom g0",
                "site s0",
                "body c0",
                "frame f",
                "composite c"
            ]
        );
    }

    #[test]
    fn an_entry_past_its_list_is_skipped_and_an_element_no_entry_reaches_comes_after() {
        let mut body = parsed_body();
        body.geoms.push(MjcfGeom {
            name: Some("g1".into()),
            ..Default::default()
        });
        // Two Body entries for one body, one Geom entry for two geoms.
        body.items = vec![MjcfItem::Body, MjcfItem::Geom, MjcfItem::Body];
        let elements = Contents::take_from_body(&mut body).into_elements();
        assert_eq!(
            names(&elements),
            [
                "body c0",
                "geom g0",
                "joint j",
                "geom g1",
                "site s0",
                "frame f",
                "composite c"
            ]
        );
    }

    #[test]
    fn pushed_elements_go_back_with_their_items() {
        let mut body = MjcfBody::new("b");
        let mut contents = Contents::default();
        contents.push(Element::Geom(MjcfGeom::default()));
        contents.push(Element::Body(MjcfBody::new("c")));
        contents.push(Element::Geom(MjcfGeom::default()));
        contents.put_into_body(&mut body);
        assert_eq!(body.items, [MjcfItem::Geom, MjcfItem::Body, MjcfItem::Geom]);
        assert_eq!((body.geoms.len(), body.children.len()), (2, 1));
    }
}
