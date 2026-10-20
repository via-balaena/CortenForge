//! The order in which MuJoCo lists contacts and the geoms of each
//! (`engine_collision_driver.c:226-470`, `:1475-1480`).
//!
//! MuJoCo walks the broadphase's body–flex pairs by signature `(bf1, bf2)`,
//! `bf1 < bf2`, a flex numbered after the bodies. Before each pair it
//! collides the explicit `<pair>`s whose signature is not past it, in their
//! compiled order, and after the last pair the rest; each flex's collisions
//! with itself come last. Within a pair of bodies, the midphase sorts the
//! contacts by their listed geoms (`contactcompare`, `:226-256`: its comment
//! says it undoes the type order, its code does not), or with the midphase
//! disabled the first body's geoms are taken in turn against the second's.
//! Each contact lists the geom of the lower `mjtGeom` first, equal types in
//! the order tested.

use crate::types::{Contact, DISABLE_MIDPHASE, Model, disabled};
use nalgebra::Vector3;

/// Where a contact falls in MuJoCo's list; contacts sort by it, stably.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(super) struct ListPlace {
    /// A flex's collision with itself, after every pair.
    flex_self: bool,
    /// The body–flex pair's signature.
    signature: (usize, usize),
    /// From the broadphase, after the explicit pairs of the same signature.
    broadphase: bool,
    /// The order within the signature.
    within: (usize, usize),
}

/// What the collision driver needs to list contacts as MuJoCo does.
pub(super) struct Lister {
    midphase: bool,
}

impl Lister {
    pub(super) fn new(model: &Model) -> Self {
        Self {
            midphase: !disabled(model, DISABLE_MIDPHASE),
        }
    }

    /// List a contact of the broadphase's geom pair `(geom1, geom2)` as
    /// MuJoCo does, and its place.
    pub(super) fn broadphase(
        &self,
        model: &Model,
        contact: &mut Contact,
        (geom1, geom2): (usize, usize),
    ) -> ListPlace {
        let tested = lower_body_first(model, (geom1, geom2));
        let listed = type_ordered(model, tested);
        list_as(contact, listed);
        ListPlace {
            flex_self: false,
            signature: bodies(model, tested),
            broadphase: true,
            within: if self.midphase { listed } else { tested },
        }
    }
}

/// List a contact of explicit pair `pair` as MuJoCo does, and its place.
/// MuJoCo compiles a pair with the geom of the lower body first
/// (`user_objects.cc:5645-5657`) and sorts the pairs by signature,
/// stably (`user_model.cc:4969-4972`): the list's order by signature,
/// then pair index, is that order.
pub(super) fn pair_place(model: &Model, contact: &mut Contact, pair: usize) -> ListPlace {
    let declared = &model.contact_pairs[pair];
    let tested = lower_body_first(model, (declared.geom1, declared.geom2));
    list_as(contact, type_ordered(model, tested));
    ListPlace {
        flex_self: false,
        signature: bodies(model, tested),
        broadphase: false,
        within: (pair, 0),
    }
}

/// The place of a flex contact: a flex vertex against a rigid geom (MuJoCo
/// takes the body's geoms in turn), two flexes, or a flex with itself. A
/// contact's flex vertices name its flexes.
pub(super) fn flex_place(model: &Model, contact: &Contact) -> ListPlace {
    let flex = |v: Option<usize>| v.map_or(0, |v| model.nbody + model.flexvert_flexid[v]);
    let f1 = flex(contact.flex_vertex);
    let (signature, flex_self, within) = if contact.flex_vertex2.is_none() {
        (
            (model.geom_body[contact.geom1], f1),
            false,
            (contact.geom1, 0),
        )
    } else {
        let f2 = flex(contact.flex_vertex2);
        ((f1.min(f2), f1.max(f2)), f1 == f2, (0, 0))
    };
    ListPlace {
        flex_self,
        signature,
        broadphase: true,
        within,
    }
}

/// `(geom1, geom2)` with the geom of the lower body first, equal bodies as
/// given.
pub(super) fn lower_body_first(model: &Model, (geom1, geom2): (usize, usize)) -> (usize, usize) {
    if model.geom_body[geom1] > model.geom_body[geom2] {
        (geom2, geom1)
    } else {
        (geom1, geom2)
    }
}

fn bodies(model: &Model, (geom1, geom2): (usize, usize)) -> (usize, usize) {
    (model.geom_body[geom1], model.geom_body[geom2])
}

/// `tested` in MuJoCo's type order: the lower `mjtGeom` first, equal types
/// as tested.
pub(super) fn type_ordered(model: &Model, tested: (usize, usize)) -> (usize, usize) {
    if model.geom_type[tested.0].mjtgeom() > model.geom_type[tested.1].mjtgeom() {
        (tested.1, tested.0)
    } else {
        tested
    }
}

/// List `contact` with the geoms `(geom1, geom2)`: if the collision function
/// listed them the other way round, swap them, turn the normal round, keep
/// the first tangent and take the second as the reversed normal × the
/// first, `make_frame`'s product: the frame `mju_makeFrame` builds from the
/// reversed normal, bit for bit (the first tangent's seed and its part along
/// the normal do not change sign; negating the second tangent instead would
/// turn a 0 it builds as +0 into −0), with the normal normalised once, as
/// MuJoCo's.
fn list_as(contact: &mut Contact, (geom1, geom2): (usize, usize)) {
    if geom1 != geom2 && (contact.geom1, contact.geom2) == (geom2, geom1) {
        contact.geom1 = geom1;
        contact.geom2 = geom2;
        contact.normal = -contact.normal;
        let (n, t1) = (contact.normal, contact.frame[0]);
        contact.frame[1] = Vector3::new(
            n.y * t1.z - n.z * t1.y,
            n.z * t1.x - n.x * t1.z,
            n.x * t1.y - n.y * t1.x,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};
    use crate::types::make_frame;

    /// A swapped contact turns its normal round, keeps the first tangent and
    /// rebuilds the second: the frame `make_frame` builds from the reversed
    /// normal, bit for bit, the signs of its zeros included ((0, 1, 1)'s
    /// second tangent has two), without normalising the normal a second time
    /// (which moves the last bit of (0, 1, 1)'s unit normal).
    #[test]
    fn swapped_contact_is_normalised_once() {
        let raw = Vector3::new(0.0, 1.0, 1.0);
        let mut contact = Contact::with_condim(
            Vector3::zeros(),
            raw,
            0.01,
            1,
            0,
            1.0,
            0.005,
            0.0001,
            3,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        list_as(&mut contact, (0, 1));
        assert_eq!((contact.geom1, contact.geom2), (0, 1));
        let (normal, frame) = make_frame(&-raw, &Vector3::zeros());
        let bits = |v: &Vector3<f64>| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
        assert_eq!(bits(&contact.normal), bits(&normal));
        assert_eq!(bits(&contact.frame[0]), bits(&frame[0]));
        assert_eq!(bits(&contact.frame[1]), bits(&frame[1]));
        assert!(
            frame[1].iter().any(|&x| x == 0.0),
            "the second tangent has no 0, so this test cannot see its sign"
        );
        assert_ne!(
            make_frame(&contact.normal, &Vector3::zeros()).0,
            contact.normal,
            "normalising the unit normal again must move it, or this test cannot see it"
        );
    }
}
