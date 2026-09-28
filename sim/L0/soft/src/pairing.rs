//! Friction as a library of contact pairings (soft-contact recon §5c, §9 decision 3, §16w).
//!
//! Friction is the least known of the fit test's inputs, uncertain by more than ten times across pairings (§5c), and
//! Jon's direction is that "different things can have different lubricants". So a pairing is what slides on what,
//! in which lubricant and after how long, with each measurement it rests on: its value or range, whether it was read
//! as sliding started (onset) or after it, its source and what the source actually measured.
//!
//! The solver's contact takes one `μ_f`. A verdict runs a pairing's [`corners`](Pairing::corners): `μ_f` 0 for the
//! geometric share of the push, and the pairing's lowest and highest values (§15h). Which of onset and sliding the
//! solver should take is not settled (§5c), so both are kept and the corners span them. No nominal value is sourced,
//! so a pairing has none (§15g's list for steps 6–9: which value D3 judges at is open).
//!
//! A lubricant's states are separate pairings, so a verdict picks one; the later ones show where use takes it.
//!
//! The damped tube's Coulomb push passes its check at `μ_f` 0.3 and fails it at 0.6 (§16p), so a corner above
//! [`FRICTION_CHECKED_TO`] is unchecked. Before a verdict there is trusted, its own Coulomb push is checked there and
//! the silicone's damping (fit plan U15) is settled; and above about 1.0 the half-space's own sliding is unstable at
//! ν 0.49, so a finer mesh need not converge there (the list for steps 6–9).
//!
//! Every pairing is silicone on skin, from §5c's table; each measurement's `conditions` say where a source measured
//! something else. Two of the table's rows are not pairings here: water alone has no measurement on skin, only on
//! analogs, and the tacky pad that held twice its load bounds friction from below without giving a value. No
//! measurement on the skin the product meets was found (§5c).

/// The highest `μ_f` whose Coulomb push is checked (soft-contact recon §16p: the damped tube passes at 0.3 and fails
/// at 0.6).
pub const FRICTION_CHECKED_TO: f64 = 0.3;

/// Whether a verdict at friction `mu` rests on a checked Coulomb push.
#[must_use]
pub fn checked(mu: f64) -> bool {
    mu <= FRICTION_CHECKED_TO
}

/// The lubricant between the two surfaces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lubricant {
    /// None: dry.
    Dry,
    /// A water-based gel.
    WaterBasedGel,
    /// A silicone lubricant.
    Silicone,
}

/// How long after the lubricant went on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// As applied.
    Fresh,
    /// This many minutes later, applied once.
    After {
        /// Minutes since it was applied.
        minutes: u32,
    },
}

/// When in a slide a coefficient was read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// The peak as sliding starts.
    Onset,
    /// After the peak, while sliding.
    Sliding,
}

/// One source's coefficient: a value (`low == high`) or a range.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measurement {
    /// When in the slide it was read.
    pub phase: Phase,
    /// The lowest value it gives.
    pub low: f64,
    /// The highest value it gives.
    pub high: f64,
    /// The source.
    pub source: &'static str,
    /// What was measured, and how the value was read.
    pub conditions: &'static str,
}

/// A contact pairing: what slides on what, in which lubricant, and what it has been measured at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pairing {
    /// Its name.
    pub name: &'static str,
    /// The lubricant.
    pub lubricant: Lubricant,
    /// How long after the lubricant went on.
    pub state: State,
    /// The measurements it rests on; at least one.
    pub measurements: &'static [Measurement],
}

impl Pairing {
    /// The lowest value any of its measurements gives.
    #[must_use]
    pub fn lowest(&self) -> f64 {
        self.measurements
            .iter()
            .map(|m| m.low)
            .fold(f64::INFINITY, f64::min)
    }

    /// The highest value any of its measurements gives.
    #[must_use]
    pub fn highest(&self) -> f64 {
        self.measurements
            .iter()
            .map(|m| m.high)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// A verdict's friction corners: `μ_f` 0, then the pairing's lowest and highest values (soft-contact recon
    /// §15h).
    #[must_use]
    pub fn corners(&self) -> [f64; 3] {
        [0.0, self.lowest(), self.highest()]
    }
}

const MASEN_2020: &str = "Masen 2020, https://doi.org/10.1371/journal.pone.0239363";

/// The pairings, from soft-contact recon §5c's table.
pub const PAIRINGS: &[Pairing] = &[
    Pairing {
        name: "silicone on skin, dry",
        lubricant: Lubricant::Dry,
        state: State::Fresh,
        measurements: &[
            Measurement {
                phase: Phase::Onset,
                low: 0.94,
                high: 0.94,
                source: MASEN_2020,
                conditions: "forearm skin, one subject, 20 kPa; read from its Fig. 3",
            },
            Measurement {
                phase: Phase::Onset,
                low: 1.14,
                high: 1.14,
                source: "Yap 2021, https://doi.org/10.1038/s41598-021-91119-0",
                conditions: "forearm skin, 7 subjects, 14 kPa; printed on its Fig. 2b",
            },
            Measurement {
                phase: Phase::Sliding,
                low: 0.40,
                high: 0.82,
                source: "Zhang & Mak 1999, https://doi.org/10.3109/03093649909071625",
                conditions: "0.61 ± 0.21 over six sites and ten subjects, kept as its mean ± that spread, not the data's range; a \
                             liner silicone of unstated grade",
            },
        ],
    },
    Pairing {
        name: "silicone on skin, water-based gel, fresh",
        lubricant: Lubricant::WaterBasedGel,
        state: State::Fresh,
        measurements: &[
            Measurement {
                phase: Phase::Onset,
                low: 0.18,
                high: 0.18,
                source: MASEN_2020,
                conditions: "forearm skin; read from its Fig. 3",
            },
            Measurement {
                phase: Phase::Sliding,
                low: 0.104,
                high: 0.145,
                source: "Watanabe 2024, https://doi.org/10.1038/s44172-024-00177-5",
                conditions: "seven gels; an endoscope's fluoropolymer tube on skin post mortem, not silicone",
            },
        ],
    },
    Pairing {
        name: "silicone on skin, water-based gel, after 5 min",
        lubricant: Lubricant::WaterBasedGel,
        state: State::After { minutes: 5 },
        measurements: &[Measurement {
            phase: Phase::Onset,
            low: 0.96,
            high: 0.96,
            source: MASEN_2020,
            conditions: "forearm skin, one subject, about 2 mg/cm² applied once: back at the dry value",
        }],
    },
    Pairing {
        name: "silicone on skin, silicone lubricant, fresh",
        lubricant: Lubricant::Silicone,
        state: State::Fresh,
        measurements: &[Measurement {
            phase: Phase::Onset,
            low: 0.30,
            high: 0.30,
            source: MASEN_2020,
            conditions: "forearm skin, on application; read from its Fig. 3",
        }],
    },
    Pairing {
        name: "silicone on skin, silicone lubricant, after 20 min",
        lubricant: Lubricant::Silicone,
        state: State::After { minutes: 20 },
        measurements: &[Measurement {
            phase: Phase::Onset,
            low: 1.21,
            high: 1.21,
            source: MASEN_2020,
            conditions: "forearm skin, applied once; read from its Fig. 3",
        }],
    },
];
