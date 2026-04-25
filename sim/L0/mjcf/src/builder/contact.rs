//! Contact pair and exclude processing.
//!
//! Converts MJCF `<contact>` elements (pairs and excludes) into Model arrays.
//! Handles geom/body name resolution, geom-combination fallbacks for unspecified
//! pair attributes, and deduplication with last-wins semantics.

use super::{ModelBuilder, ModelConversionError};
use crate::types::MjcfContact;
use sim_core::ContactPair;

impl ModelBuilder {
    /// Process `<contact>` element: resolve `<pair>` and `<exclude>` entries.
    ///
    /// For each `<pair>`: apply defaults class, resolve geom names to indices,
    /// compute geom-combination fallbacks for unspecified attributes, and build
    /// a fully-resolved `ContactPair`. Duplicate pairs (same geom pair) use
    /// last-wins semantics.
    ///
    /// For each `<exclude>`: resolve body names to indices and insert into the
    /// exclude set.
    pub(crate) fn process_contact(
        &mut self,
        contact: &MjcfContact,
    ) -> std::result::Result<(), ModelConversionError> {
        // Process pairs
        for mjcf_pair in &contact.pairs {
            // Stage 1: apply defaults class
            let pair = self.resolver.apply_to_pair(mjcf_pair);

            // Resolve geom names to indices
            let g1 =
                *self
                    .geom_name_to_id
                    .get(&pair.geom1)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "<pair> references unknown geom '{}' in geom1",
                            pair.geom1
                        ),
                    })?;
            let g2 =
                *self
                    .geom_name_to_id
                    .get(&pair.geom2)
                    .ok_or_else(|| ModelConversionError {
                        message: format!(
                            "<pair> references unknown geom '{}' in geom2",
                            pair.geom2
                        ),
                    })?;

            // Stage 2: geom-combination fallbacks for unspecified attributes
            let condim = pair
                .condim
                .unwrap_or_else(|| self.geom_condim[g1].max(self.geom_condim[g2]));

            let friction = pair.friction.unwrap_or_else(|| {
                let f1 = self.geom_friction[g1];
                let f2 = self.geom_friction[g2];
                // Geometric mean in 3D, then expand to 5D: [s, s, t, r, r]
                let s = (f1.x * f2.x).sqrt();
                let t = (f1.y * f2.y).sqrt();
                let r = (f1.z * f2.z).sqrt();
                [s, s, t, r, r]
            });

            let solref = pair.solref.unwrap_or_else(|| {
                // Element-wise min (our standing approximation)
                [
                    self.geom_solref[g1][0].min(self.geom_solref[g2][0]),
                    self.geom_solref[g1][1].min(self.geom_solref[g2][1]),
                ]
            });

            let solimp = pair.solimp.unwrap_or_else(|| {
                // Element-wise max (our standing approximation)
                let s1 = self.geom_solimp[g1];
                let s2 = self.geom_solimp[g2];
                [
                    s1[0].max(s2[0]),
                    s1[1].max(s2[1]),
                    s1[2].max(s2[2]),
                    s1[3].max(s2[3]),
                    s1[4].max(s2[4]),
                ]
            });

            // solreffriction: [0,0] sentinel means "use solref" (MuJoCo convention).
            // Only explicit <pair solreffriction="..."/> sets a nonzero value.
            let solreffriction = pair.solreffriction.unwrap_or([0.0, 0.0]);

            // margin/gap: geom-level not yet parsed, default to 0.0
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

            // Deduplicate: canonical key (min, max) for symmetry
            let key = (g1.min(g2), g1.max(g2));
            if self.contact_pair_set.contains(&key) {
                // Last-wins: replace existing entry
                let pos = self
                    .contact_pairs
                    .iter()
                    .position(|p| (p.geom1.min(p.geom2), p.geom1.max(p.geom2)) == key)
                    .ok_or_else(|| ModelConversionError {
                        message: "contact_pair_set and contact_pairs out of sync".into(),
                    })?;
                self.contact_pairs[pos] = contact_pair;
            } else {
                self.contact_pair_set.insert(key);
                self.contact_pairs.push(contact_pair);
            }
        }

        // Process excludes
        for mjcf_exclude in &contact.excludes {
            let b1 = *self
                .body_name_to_id
                .get(&mjcf_exclude.body1)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "<exclude> references unknown body '{}' in body1",
                        mjcf_exclude.body1
                    ),
                })?;
            let b2 = *self
                .body_name_to_id
                .get(&mjcf_exclude.body2)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "<exclude> references unknown body '{}' in body2",
                        mjcf_exclude.body2
                    ),
                })?;
            // Canonical key for symmetry
            self.contact_excludes.insert((b1.min(b2), b1.max(b2)));
        }

        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use crate::builder::load_model;
    use approx::assert_relative_eq;

    /// `<pair>` with explicit `friction`/`condim`/`solref`/`solimp` skips
    /// the geom-combination fallbacks; the supplied values land verbatim
    /// in the resulting `ContactPair`.
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

    /// `<pair>` with no overrides applies geom-combination fallbacks per
    /// `process_contact`: condim = max, friction = sqrt(componentwise) ×
    /// 5-element expansion `[s,s,t,r,r]`, solref = elementwise min, solimp
    /// = elementwise max. Each side carries distinct values so the asserts
    /// fail under any other commutative combinator (e.g., max-vs-min swap).
    #[test]
    fn pair_geom_combination_fallbacks_applied() {
        // Distinct per-geom values that make the combination behavior
        // observable: friction means are non-trivial; solref min picks
        // [0.01, 1.0] from g1; solimp max picks [0.9, 0.95, 0.005, 0.6,
        // 3.0] (componentwise across the two geoms — not all from one
        // side, so a "first-wins" or "last-wins" implementation would
        // fail too).
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
        // condim: max(3, 6) = 6.
        assert_eq!(p.condim, 6);
        // friction: geometric mean per component, then expand to [s,s,t,r,r].
        let s = (1.0_f64 * 4.0).sqrt(); // 2.0
        let t = (0.04_f64 * 0.16).sqrt(); // 0.08
        let r = (0.001_f64 * 0.004).sqrt(); // ≈0.002
        for (got, want) in p.friction.iter().zip([s, s, t, r, r].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
        // solref: elementwise min. min(0.01, 0.05) = 0.01; min(1.0, 2.0) = 1.0.
        // Both come from g1 — a swapped (max-instead-of-min) implementation
        // would yield [0.05, 2.0] and fail the first assert.
        for (got, want) in p.solref.iter().zip([0.01, 1.0].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
        // solimp: elementwise max. Components 2 and 3 come from g1 (0.005,
        // 0.6); components 0, 1, 4 come from g2 (0.9, 0.95, 3.0) —
        // mixed-side outcome rules out any "single-side wins" alias.
        for (got, want) in p.solimp.iter().zip([0.9, 0.95, 0.005, 0.6, 3.0].iter()) {
            assert_relative_eq!(got, want, epsilon = 1e-12);
        }
    }

    /// Two `<pair>` elements with the same geom pair (in either order) are
    /// deduplicated under the canonical (min, max) key with last-wins
    /// semantics; only the most-recently-set attributes survive.
    #[test]
    fn pair_dedup_last_wins_under_canonical_key() {
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
        assert_eq!(model.contact_pairs.len(), 1, "duplicates collapsed");
        assert_eq!(model.contact_pairs[0].condim, 6, "last-wins");
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

    /// Both `<pair>` and `<exclude>` resolve names to ids and surface a
    /// `ModelConversionError` containing the offending name when the
    /// referenced geom or body doesn't exist.
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
        assert!(err.to_string().contains("ghost"));

        let excl_xml = r#"
            <mujoco model="m">
                <worldbody>
                    <body name="a"><geom type="sphere" size="0.1" mass="1.0"/></body>
                </worldbody>
                <contact><exclude body1="a" body2="ghost"/></contact>
            </mujoco>
        "#;
        let err = load_model(excl_xml).expect_err("unknown body2");
        assert!(err.to_string().contains("ghost"));
    }
}
