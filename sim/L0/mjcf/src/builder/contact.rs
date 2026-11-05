//! Contact pair and exclude processing.
//!
//! Converts MJCF `<contact>` elements (pairs and excludes) into Model arrays.
//! Handles geom/body name resolution, a pair's unwritten values from its class
//! or MuJoCo's pair defaults, every pair kept, a repeated one too, and the
//! pairs stored in MuJoCo's order.

use super::{DEFAULT_SOLIMP, DEFAULT_SOLREF, ModelBuilder};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::MjcfContact;
use sim_core::ContactPair;

/// MuJoCo's default `<pair friction>` (`user_init.c` `mjs_defaultPair`).
const PAIR_FRICTION: [f64; 5] = [1.0, 1.0, 0.005, 0.0001, 0.0001];

impl ModelBuilder {
    /// Process `<contact>` element: resolve `<pair>` and `<exclude>` entries.
    ///
    /// For each `<pair>`: resolve geom names to indices and build a
    /// `ContactPair` whose unwritten values are its class's, else MuJoCo's
    /// pair defaults. A pair on the geoms of an earlier one is kept beside
    /// it, as MuJoCo 3.5.0 keeps it. Each pair names the lower body's geom
    /// first, and the pairs are sorted by their bodies, stably, as MuJoCo
    /// compiles them.
    ///
    /// For each `<exclude>`: resolve body names to indices and insert into the
    /// exclude set.
    pub(crate) fn process_contact(&mut self, contact: &MjcfContact) -> Result<()> {
        // Process pairs
        for (i, mjcf_pair) in contact.pairs.iter().enumerate() {
            // Its class applied already (`resolve::apply_defaults`).
            let pair = mjcf_pair;
            let at = || Location::nth(ElementKind::Pair, pair.name.as_deref(), i);

            // A geom named "" where the model has an unnamed geom, which
            // MuJoCo binds to it, is refused after the keyframes
            // ([`check_pair_geom_names`]).
            if pair.geom1.is_empty() || pair.geom2.is_empty() {
                continue;
            }
            // Validation found both (`check_pairs`); `discardvisual` may
            // have removed one since (ledger L342).
            let g1 = *self
                .geom_name_to_id
                .get(&pair.geom1)
                .ok_or_else(|| MjcfError::undefined(ElementKind::Geom, &pair.geom1).at(at()))?;
            let g2 = *self
                .geom_name_to_id
                .get(&pair.geom2)
                .ok_or_else(|| MjcfError::undefined(ElementKind::Geom, &pair.geom2).at(at()))?;
            // The geom of the lower body first, as MuJoCo compiles a pair
            // (`mjCPair::ResolveReferences`, `user_objects.cc:5645-5654`).
            let (g1, g2) = if self.geom_body[g1] > self.geom_body[g2] {
                (g2, g1)
            } else {
                (g1, g2)
            };

            // A value its class or the pair writes lies over MuJoCo's pair
            // default (`mjs_defaultPair`, `user_init.c:288-297`; measured on
            // 3.5.0: `friction="0.7"` is 0.7 1 0.005 0.0001 0.0001); an
            // unwritten one is the default, never the two geoms' (ledger
            // L147, L241): `mjCPair::Compile` takes the geoms' value only
            // for one left undefined, which its XML reader leaves only where
            // `nan` is written (measured), and this crate refuses `nan`
            // (`D-NONFINITE`).
            let condim = pair.condim.unwrap_or(3);
            let friction = pair
                .friction
                .map_or(PAIR_FRICTION, |f| f.over(PAIR_FRICTION));
            let solref = pair
                .solref
                .map_or(DEFAULT_SOLREF, |s| s.over(DEFAULT_SOLREF));
            let solimp = pair
                .solimp
                .map_or(DEFAULT_SOLIMP, |s| s.over(DEFAULT_SOLIMP));

            // solreffriction: [0,0] sentinel means "use solref" (MuJoCo convention).
            // Only explicit <pair solreffriction="..."/> sets a nonzero value.
            let solreffriction = pair
                .solreffriction
                .map_or([0.0, 0.0], |s| s.over([0.0, 0.0]));

            // margin/gap: MuJoCo's pair default 0, not the geoms'.
            let margin = pair.margin.unwrap_or(0.0);
            let gap = pair.gap.unwrap_or(0.0);

            let contact_pair = ContactPair {
                geom1: g1,
                geom2: g2,
                condim,
                friction,
                solref,
                solreffriction,
                solimp,
                margin,
                gap,
            };

            // Each pair is kept, as MuJoCo keeps it, and each makes its own
            // contacts; the set holds a geom pair's key once, for the
            // automatic contacts an explicit pair replaces.
            self.contact_pair_set.insert((g1.min(g2), g1.max(g2)));
            self.contact_pairs.push(contact_pair);
        }
        // By their bodies, stably: MuJoCo's order (`user_model.cc:4969-4972`,
        // signature `(body1 << 16) + body2`, `user_objects.cc:5657`).
        let geom_body = &self.geom_body;
        self.contact_pairs
            .sort_by_key(|pair| (geom_body[pair.geom1], geom_body[pair.geom2]));

        // Process excludes
        for (i, mjcf_exclude) in contact.excludes.iter().enumerate() {
            let body = |name: &str| {
                self.body_name_to_id.get(name).copied().ok_or_else(|| {
                    MjcfError::undefined(ElementKind::Body, name).at(Location::nth(
                        ElementKind::Exclude,
                        mjcf_exclude.name.as_deref(),
                        i,
                    ))
                })
            };
            let b1 = body(&mjcf_exclude.body1)?;
            let b2 = body(&mjcf_exclude.body2)?;
            // Canonical key for symmetry
            self.contact_excludes.insert((b1.min(b2), b1.max(b2)));
        }

        Ok(())
    }
}

/// A pair naming a geom "", which MuJoCo 3.5.0 binds to an unnamed geom of
/// the model (registry `D-PAIR-NO-GEOM`): refused, the first in order, after
/// the keyframes with the other refusals MuJoCo does not make. Where the
/// model has no unnamed geom, MuJoCo refuses the pair too and validation has
/// refused it where the pairs are checked.
pub fn check_pair_geom_names(contact: &MjcfContact) -> Result<()> {
    for (i, pair) in contact.pairs.iter().enumerate() {
        if let Some(geom) = [&pair.geom1, &pair.geom2]
            .into_iter()
            .find(|g| g.is_empty())
        {
            return Err(
                MjcfError::undefined(ElementKind::Geom, geom).at(Location::nth(
                    ElementKind::Pair,
                    pair.name.as_deref(),
                    i,
                )),
            );
        }
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use crate::builder::load_model;
    use crate::error::{ElementKind, MjcfError};
    use approx::assert_relative_eq;

    /// `<pair>` with explicit `friction`/`condim`/`solref`/`solimp`: the
    /// supplied values land verbatim in the resulting `ContactPair`.
    #[test]
    fn pair_explicit_attrs_pass_through() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom name="g1" type="sphere" size="0.1" mass="1.0"/></body>
                    <body name="b"><geom name="g2" type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody>
                <contact>
                    <pair geom1="g1" geom2="g2"
                          condim="6"
                          friction="0.7 0.7 0.05 0.001 0.001"
                          solref="0.03 1.2"
                          solimp="0.95 0.99 0.002 0.6 3.0"
                          solreffriction="0.04 1.1"
                          margin="0.01" gap="0.005"/>
                </contact>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.contact_pairs.len(), 1);
        let p = &model.contact_pairs[0];
        assert_eq!(p.condim, 6);
        for (got, want) in p.friction.iter().zip([0.7, 0.7, 0.05, 0.001, 0.001].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
        for (got, want) in p.solref.iter().zip([0.03, 1.2].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
        for (got, want) in p.solimp.iter().zip([0.95, 0.99, 0.002, 0.6, 3.0].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
        for (got, want) in p.solreffriction.iter().zip([0.04, 1.1].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
        assert_relative_eq!(p.margin, 0.01, epsilon = 1e-12);
        assert_relative_eq!(p.gap, 0.005, epsilon = 1e-12);
    }

    /// `<pair>` with no overrides takes MuJoCo 3.5.0's pair defaults, not its
    /// geoms' values: each geom writes its own condim, friction, solref and
    /// solimp, and MuJoCo gives this pair condim 3 and the defaults
    /// (measured).
    #[test]
    fn pair_unwritten_values_are_mujoco_s_defaults() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom name="g1" type="sphere" size="0.1" mass="1.0"
                        condim="3" friction="1 0.04 0.001"
                        solref="0.01 1.0"
                        solimp="0.7 0.8 0.005 0.4 1.0"/></body>
                    <body name="b"><geom name="g2" type="sphere" size="0.1" mass="1.0"
                        condim="6" friction="4 0.16 0.004"
                        solref="0.05 2.0"
                        solimp="0.9 0.95 0.001 0.6 3.0"/></body>
                </worldbody>
                <contact>
                    <pair geom1="g1" geom2="g2"/>
                </contact>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        let p = &model.contact_pairs[0];
        assert_eq!(p.condim, 3);
        assert_eq!(
            p.friction.map(f64::to_bits),
            super::PAIR_FRICTION.map(f64::to_bits)
        );
        assert_eq!(
            p.solref.map(f64::to_bits),
            super::DEFAULT_SOLREF.map(f64::to_bits)
        );
        assert_eq!(
            p.solimp.map(f64::to_bits),
            super::DEFAULT_SOLIMP.map(f64::to_bits)
        );
    }

    /// Two `<pair>` elements on the same geoms (in either order) are both
    /// kept, in the order written, as MuJoCo 3.5.0 keeps them (measured:
    /// npair 2, condims 3 and 6).
    #[test]
    fn repeated_pairs_both_kept() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom name="g1" type="sphere" size="0.1" mass="1.0"/></body>
                    <body name="b"><geom name="g2" type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody>
                <contact>
                    <pair geom1="g1" geom2="g2" condim="3"/>
                    <pair geom1="g2" geom2="g1" condim="6"/>
                </contact>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        let condims: Vec<i32> = model.contact_pairs.iter().map(|p| p.condim).collect();
        assert_eq!(condims, [3, 6]);
        assert_eq!(model.contact_pair_set.len(), 1);
    }

    /// `<exclude>` resolves both body names and inserts under a canonical
    /// (min, max) key; specifying the same pair in opposite order is a
    /// no-op duplicate.
    #[test]
    fn exclude_canonical_key_dedups() {
        let xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom type="sphere" size="0.1" mass="1.0"/></body>
                    <body name="b"><geom type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody>
                <contact>
                    <exclude body1="a" body2="b"/>
                    <exclude body1="b" body2="a"/>
                </contact>
            </mujoco>
        "#;
        let model = load_model(xml).expect("load");
        assert_eq!(model.contact_excludes.len(), 1);
        let a = *model.body_name_to_id.get("a").unwrap();
        let b = *model.body_name_to_id.get("b").unwrap();
        let key = (a.min(b), a.max(b));
        assert!(model.contact_excludes.contains(&key));
    }

    /// Both `<pair>` and `<exclude>` resolve names to ids and surface an
    /// [`MjcfError::UndefinedReference`] naming the geom or body that doesn't
    /// exist.
    #[test]
    fn contact_unknown_name_errors() {
        let pair_xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom name="g1" type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody>
                <contact><pair geom1="g1" geom2="ghost"/></contact>
            </mujoco>
        "#;
        let err = load_model(pair_xml).expect_err("unknown geom2");
        assert!(
            matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Geom, name, .. }
                if name == "ghost"),
            "{err}"
        );

        let excl_xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody>
                <contact><exclude body1="a" body2="ghost"/></contact>
            </mujoco>
        "#;
        let err = load_model(excl_xml).expect_err("unknown body2");
        assert!(
            matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Body, name, .. }
                if name == "ghost"),
            "{err}"
        );
    }
}
