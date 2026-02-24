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
