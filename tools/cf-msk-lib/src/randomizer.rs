//! `RandomizerSource` — sample a **population** of leg twins from the validated
//! anthropometric family (free training data, no scan).
//!
//! The A4 move (recon `03_phases/leg_region`): turn the dial-able [`AnthroSource`]
//! generator into a *sampler* over its parameter space, so we get a population of
//! plausible bodies to train / stress / cover-check against. It does **not** invent
//! a new morph — it draws `(sex, stature percentile, girth percentile)` and hands
//! them to [`AnthroSource`], reusing the whole validated stack (the per-axis morph
//! that matches OpenSim's ScaleTool, the published proportions, the provenance).
//! That is why a sampled body carries [`AnthroSource::is_coupled`] provenance, so
//! the scorecard grades it with the exact regime, never inferring.
//!
//! ## Sampling regime (the product-intent call, confirmed 2026-06-08)
//!
//! **Coupled-by-default + a reported decoupled tail.** Most draws are *coupled*
//! (girth tracks stature — the validated ≥0.95 shape-corr regime); a tunable
//! fraction are *bounded decoupled* builds (stature/girth dialed apart, the
//! tall-lean / short-stocky anisotropy real people have) that the scorecard
//! **reports** against the loose floor rather than hard-gates. This covers the
//! harder anisotropic regime for training while staying honest: the population
//! validates **coverage + the machinery, not personhood** (see the scorecard's
//! tier-applicability matrix). Stature/girth percentiles are sampled **uniform on
//! the percentile axis** — which, since a percentile *is* the population CDF
//! coordinate, reproduces the underlying measurement distribution by rank — within
//! the validated `(pct_lo, pct_hi)` range.
//!
//! The PRNG is a tiny seeded **SplitMix64** ([`Rng`]) so populations are exactly
//! reproducible and the crate stays dependency-free (no `rand`).

use crate::anthro::{AnthroSource, Sex};

/// A seeded **SplitMix64** PRNG — deterministic, dependency-free, good enough for
/// sampling a body population (this is not cryptographic). Same seed ⇒ same stream.
#[derive(Debug, Clone)]
pub struct Rng {
    state: u64,
}

impl Rng {
    /// A generator seeded with `seed`.
    pub fn seed(seed: u64) -> Self {
        Rng { state: seed }
    }

    /// The next 64 raw bits (the SplitMix64 step).
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A uniform `f64` in `[0, 1)` (53-bit mantissa).
    pub fn unit(&mut self) -> f64 {
        // Top 53 bits → [0, 2^53) → scale into [0, 1).
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// A uniform `f64` in `[lo, hi)`.
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

/// Knobs for the population sampler (sensible defaults via [`Default`]).
#[derive(Debug, Clone, Copy)]
pub struct RandomizerConfig {
    /// Lowest sampled percentile (open-interval floor). Default `0.01`.
    pub pct_lo: f64,
    /// Highest sampled percentile (open-interval ceiling). Default `0.99`.
    pub pct_hi: f64,
    /// Fraction of draws that are *decoupled* (girth dialed apart from stature).
    /// Default `0.2` — a minority tail for anisotropic coverage.
    pub decoupled_fraction: f64,
    /// Max `|stature_pct − girth_pct|` for a decoupled draw (keeps the tail bounded,
    /// not pathological). Default `0.5`.
    pub max_decoupling: f64,
    /// P(female) for each draw. Default `0.5`.
    pub female_fraction: f64,
}

impl Default for RandomizerConfig {
    fn default() -> Self {
        RandomizerConfig {
            pct_lo: 0.01,
            pct_hi: 0.99,
            decoupled_fraction: 0.2,
            max_decoupling: 0.5,
            female_fraction: 0.5,
        }
    }
}

impl RandomizerConfig {
    fn validate(&self) {
        assert!(
            self.pct_lo > 0.0 && self.pct_hi < 1.0 && self.pct_lo < self.pct_hi,
            "percentile bounds must satisfy 0 < pct_lo < pct_hi < 1 ({}, {})",
            self.pct_lo,
            self.pct_hi
        );
        assert!(
            (0.0..=1.0).contains(&self.decoupled_fraction),
            "decoupled_fraction must be in [0,1] ({})",
            self.decoupled_fraction
        );
        assert!(
            self.max_decoupling >= 0.0,
            "max_decoupling must be ≥ 0 ({})",
            self.max_decoupling
        );
        assert!(
            (0.0..=1.0).contains(&self.female_fraction),
            "female_fraction must be in [0,1] ({})",
            self.female_fraction
        );
    }
}

/// A population sampler over the validated [`AnthroSource`] family.
///
/// `RandomizerSource` chooses the percentiles; `AnthroSource` does the validated
/// morph — so each sampled body is a member of the same family the scorecard already
/// grades, with provenance intact. Draw one body with [`sample`](Self::sample) (you
/// own the [`Rng`]) or a whole reproducible population with
/// [`population`](Self::population).
#[derive(Debug, Clone, Copy, Default)]
pub struct RandomizerSource {
    config: RandomizerConfig,
}

impl RandomizerSource {
    /// A sampler with default [`RandomizerConfig`].
    pub fn new() -> Self {
        RandomizerSource::default()
    }

    /// A sampler with a custom config.
    pub fn with_config(config: RandomizerConfig) -> Self {
        config.validate();
        RandomizerSource { config }
    }

    /// The config in force.
    pub fn config(&self) -> RandomizerConfig {
        self.config
    }

    /// Draw one body (an [`AnthroSource`] carrying its coupled/decoupled provenance).
    /// You supply the [`Rng`] so draws compose into any stream.
    pub fn sample(&self, rng: &mut Rng) -> AnthroSource {
        let c = &self.config;
        let sex = if rng.unit() < c.female_fraction {
            Sex::Female
        } else {
            Sex::Male
        };
        let stature_pct = rng.range(c.pct_lo, c.pct_hi);
        if rng.unit() < c.decoupled_fraction {
            // Bounded decoupling: offset the girth percentile within ±max_decoupling,
            // clamped back into the sampled range. (A tiny offset is honestly ~coupled
            // and `is_coupled()` will report it as such — the fraction is the *attempt*
            // rate, the provenance is exact.)
            let delta = rng.range(-c.max_decoupling, c.max_decoupling);
            let girth_pct = (stature_pct + delta).clamp(c.pct_lo, c.pct_hi);
            AnthroSource::new(sex, stature_pct).with_girth_percentile(girth_pct)
        } else {
            AnthroSource::new(sex, stature_pct)
        }
    }

    /// A reproducible population of `n` bodies from `seed`. Same `(seed, n, config)`
    /// ⇒ identical population (the training-data regeneration guarantee).
    pub fn population(&self, seed: u64, n: usize) -> Vec<AnthroSource> {
        let mut rng = Rng::seed(seed);
        (0..n).map(|_| self.sample(&mut rng)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_is_deterministic_and_in_unit_range() {
        let mut a = Rng::seed(42);
        let mut b = Rng::seed(42);
        for _ in 0..1000 {
            let x = a.unit();
            assert_eq!(x, b.unit(), "same seed must reproduce the stream");
            assert!((0.0..1.0).contains(&x), "unit() out of [0,1): {x}");
        }
        // A different seed gives a different stream.
        assert_ne!(Rng::seed(1).unit(), Rng::seed(2).unit());
    }

    #[test]
    fn population_is_reproducible_and_seed_sensitive() {
        let r = RandomizerSource::new();
        let p1 = r.population(7, 64);
        let p2 = r.population(7, 64);
        let p3 = r.population(8, 64);
        assert_eq!(p1.len(), 64);
        let key = |s: &AnthroSource| (s.sex, s.stature_percentile, s.girth_percentile);
        assert!(
            p1.iter().zip(&p2).all(|(a, b)| key(a) == key(b)),
            "same seed must reproduce the population"
        );
        assert!(
            p1.iter().zip(&p3).any(|(a, b)| key(a) != key(b)),
            "a different seed must change the population"
        );
    }

    #[test]
    fn sampled_percentiles_stay_in_bounds() {
        let r = RandomizerSource::new();
        let c = r.config();
        for s in r.population(123, 500) {
            assert!(
                s.stature_percentile >= c.pct_lo && s.stature_percentile <= c.pct_hi,
                "stature pct {} out of bounds",
                s.stature_percentile
            );
            assert!(
                s.girth_percentile >= c.pct_lo && s.girth_percentile <= c.pct_hi,
                "girth pct {} out of bounds",
                s.girth_percentile
            );
        }
    }

    #[test]
    fn decoupled_fraction_is_respected_within_tolerance() {
        // A population at the default 20% decoupled fraction should land near 20%
        // (counting bodies whose provenance is actually decoupled).
        let r = RandomizerSource::new();
        let pop = r.population(2024, 2000);
        let decoupled = pop.iter().filter(|s| !s.is_coupled()).count();
        let frac = decoupled as f64 / pop.len() as f64;
        assert!(
            (0.15..0.25).contains(&frac),
            "decoupled fraction {frac:.3} far from the configured 0.20"
        );
    }

    #[test]
    fn coupled_only_config_produces_no_decoupled() {
        let r = RandomizerSource::with_config(RandomizerConfig {
            decoupled_fraction: 0.0,
            ..RandomizerConfig::default()
        });
        assert!(r.population(5, 300).iter().all(AnthroSource::is_coupled));
    }

    #[test]
    fn female_fraction_zero_is_all_male() {
        let r = RandomizerSource::with_config(RandomizerConfig {
            female_fraction: 0.0,
            ..RandomizerConfig::default()
        });
        assert!(r.population(9, 200).iter().all(|s| s.sex == Sex::Male));
    }

    #[test]
    #[should_panic(expected = "percentile bounds")]
    fn rejects_bad_percentile_bounds() {
        let _ = RandomizerSource::with_config(RandomizerConfig {
            pct_lo: 0.0,
            ..RandomizerConfig::default()
        });
    }
}
