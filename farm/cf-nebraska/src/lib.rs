//! Nebraska OECD Tractor Test figures, as a calibration oracle for the
//! acres-per-season chain.
//!
//! MISSION's headline number — *how many acres per season can one farm run on
//! its own wind-derived hydrogen* — runs through a tractor, and the tractor
//! half is only validated if it is checked against a machine somebody actually
//! put on a dynamometer. That is what this crate holds.
//!
//! # The problem this crate exists to solve
//!
//! The individual 30-page test reports are served only through a CGI endpoint
//! that refuses automated retrieval. What *is* retrievable is the Laboratory's
//! annual compilation, and the copies that can be fetched are **scans**: the
//! text layer is OCR, and OCR of a numeric table is exactly the kind of input
//! that produces a wrong number which looks right.
//!
//! Two kinds of redundancy already in the sources do the verifying, and they
//! catch different errors:
//!
//! 1. **Each figure is printed twice** — once in US customary units, once in
//!    SI. A misread digit almost never agrees with its own conversion, so the
//!    two printed forms check each other. See [`Printed::internally_consistent`].
//! 2. **The same table is reprinted every year.** A tractor tested once in 2014
//!    appears in every compilation afterwards, independently scanned each time.
//!    This catches what (1) cannot: a figure taken from the **wrong row**. A
//!    neighbouring row's value reconciles perfectly — it is a real figure,
//!    simply the wrong one — and adjacent rows here differ by as little as
//!    0.17%. See [`reconcile`].
//!
//! Three of this tractor's figures are corrupt in the 2019 scan and were
//! resolved by the 2016 and 2017 editions outvoting it; see [`CORRUPT_SCANS`],
//! which records which edition failed and on what, because that is evidence
//! about a scan rather than about the tractor.
//!
//! ⚠ Neither check says anything about whether the Laboratory **measured**
//! correctly, or mis-printed a figure in every edition. They validate
//! transcription, which is the error these sources actually introduce.
//!
//! # What this crate deliberately does NOT contain
//!
//! See [`ABSENT`]. No edition carries a wheel-slip column, none states
//! the surface the drawbar tests were run on, and the engine speed at PTO
//! maximum power is carried in one unit only. All three are recorded with their
//! reasons, because the tempting move — supplying a plausible value from
//! general knowledge — would put an unsourced number into the one part of the
//! chain that exists to be validated.
//!
//! [`NOT_TRANSCRIBED`] is the separate list: rows the source *does* carry that
//! were skipped for relevance, so the transcribed count cannot be mistaken for
//! the whole column.

/// A unit conversion from a US customary quantity to its SI counterpart.
///
/// Each variant's [`Conversion::factor`] is the exact defined ratio, not a
/// rounded one: the whole point of the reconciliation check is that the only
/// slack is the source's own printing precision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Conversion {
    /// Horsepower to kilowatts.
    HpToKw,
    /// Miles per hour to kilometres per hour.
    MphToKmh,
    /// US gallons per hour to litres per hour.
    GalPerHrToLPerH,
    /// US gallons per minute to litres per minute.
    GalPerMinToLPerMin,
    /// Horsepower-hours per US gallon to kilowatt-hours per litre.
    HpHrPerGalToKwhPerL,
    /// Pounds to kilograms.
    LbToKg,
    /// Pounds-force to kilonewtons.
    LbfToKn,
}

/// Horsepower per kilowatt, exact by definition of the foot-pound-force.
const HP_TO_KW: f64 = 0.745_699_871_582_270_2;
/// Litres per US gallon, exact by definition of the inch.
const GAL_TO_L: f64 = 3.785_411_784;
/// Newtons per pound-force, exact by definition of standard gravity.
const LBF_TO_N: f64 = 4.448_221_615_260_5;

impl Conversion {
    /// The exact multiplier taking the US value to the SI value.
    #[must_use]
    pub const fn factor(self) -> f64 {
        match self {
            Self::HpToKw => HP_TO_KW,
            Self::MphToKmh => 1.609_344,
            Self::GalPerHrToLPerH | Self::GalPerMinToLPerMin => GAL_TO_L,
            Self::HpHrPerGalToKwhPerL => HP_TO_KW / GAL_TO_L,
            Self::LbToKg => 0.453_592_37,
            Self::LbfToKn => LBF_TO_N / 1000.0,
        }
    }
}

/// Half the value of the last printed digit — the rounding slack on a figure
/// printed to `decimals` decimal places.
///
/// A figure printed as `63.5` stands for a true value anywhere in
/// `63.5 ± 0.05`; printed as `63.50` it would stand for `63.50 ± 0.005`. The
/// distinction is not pedantic: reading `60.6` as though it had been printed
/// `60.60` narrows the interval tenfold and manufactures a disagreement that
/// is not in the source. That happened once while this crate was being
/// written, which is why the precision is stored rather than inferred.
#[must_use]
fn half_ulp(decimals: u8) -> f64 {
    0.5 * 10f64.powi(-i32::from(decimals))
}

/// One figure, as printed in both unit systems by the source document.
///
/// Both values are transcribed exactly as they appear, including how many
/// decimal places they were printed to. Nothing here is normalised, because
/// the printed precision is what makes the reconciliation check meaningful.
#[derive(Clone, Copy, Debug)]
pub struct Printed {
    us: f64,
    us_decimals: u8,
    si: f64,
    si_decimals: u8,
    conversion: Conversion,
}

impl Printed {
    /// Record a figure exactly as the source printed it, in both unit systems.
    ///
    /// `us_decimals` and `si_decimals` are the decimal places each was printed
    /// to, which set the rounding slack and therefore how much the
    /// reconciliation check can resolve. They are stored rather than inferred:
    /// reading `60.6` as though it were `60.60` narrows the interval tenfold
    /// and manufactures a disagreement the source does not contain.
    #[must_use]
    pub const fn new(
        us: f64,
        us_decimals: u8,
        si: f64,
        si_decimals: u8,
        conversion: Conversion,
    ) -> Self {
        Self {
            us,
            us_decimals,
            si,
            si_decimals,
            conversion,
        }
    }

    /// The US customary figure exactly as the source printed it.
    ///
    /// ⚠ This is **transcription provenance, not a validated measurement**. It
    /// is whatever is on the page, including for a figure whose two printed
    /// forms disagree. The validated value comes from [`Datum::read`], which
    /// weighs every edition and cannot hand back a number the sources do not
    /// agree on.
    #[must_use]
    pub const fn as_printed_us(self) -> f64 {
        self.us
    }

    /// The SI figure exactly as the source printed it.
    ///
    /// ⚠ Provenance, not a validated measurement — see
    /// [`Printed::as_printed_us`].
    #[must_use]
    pub const fn as_printed_si(self) -> f64 {
        self.si
    }

    /// Decimal places the US figure was printed to.
    #[must_use]
    pub const fn us_decimals(self) -> u8 {
        self.us_decimals
    }

    /// Decimal places the SI figure was printed to.
    #[must_use]
    pub const fn si_decimals(self) -> u8 {
        self.si_decimals
    }

    /// The conversion relating the two printed forms.
    #[must_use]
    pub const fn conversion(self) -> Conversion {
        self.conversion
    }

    /// The same figure with its US value moved by a relative amount.
    ///
    /// Exists so the reconciliation check can be probed against the real data
    /// rather than a fixture: a gate nobody has made fail is vacuous, and the
    /// way to make this one fail is to move a digit and watch it stop
    /// reconciling.
    #[must_use]
    pub fn perturbed(self, relative: f64) -> Self {
        Self {
            us: self.us * (1.0 + relative),
            ..self
        }
    }

    /// The combined relative rounding slack of the two printed forms.
    ///
    /// The floor on what the reconciliation check could ever resolve: an
    /// apparent disagreement smaller than this is explained by rounding alone
    /// and says nothing about the transcription. Used to state how far a
    /// disputed figure's disagreement exceeds what rounding can account for,
    /// so that claim has a producer instead of being a number in a sentence.
    #[must_use]
    pub fn printing_slack(self) -> f64 {
        half_ulp(self.us_decimals) / self.us.abs() + half_ulp(self.si_decimals) / self.si.abs()
    }

    /// Whether this edition's two printed forms reconcile with each other.
    ///
    /// The printed US figure stands for a true value within its own rounding
    /// interval; converting that whole interval gives the range of SI values
    /// consistent with it. If that range overlaps the printed SI figure's own
    /// rounding interval, the two agree.
    #[must_use]
    pub fn internally_consistent(&self) -> bool {
        let k = self.conversion.factor();
        let u = half_ulp(self.us_decimals);
        let s = half_ulp(self.si_decimals);
        let (lo, hi) = ((self.us - u) * k, (self.us + u) * k);
        !(hi < self.si - s || lo > self.si + s)
    }

    /// The smallest relative error in the US figure the within-edition check is
    /// guaranteed to catch, or `None` if the two printed forms already
    /// disagree.
    ///
    /// The check's power is set by how many *significant* figures the source
    /// printed, not how many decimal places. `227.60 hp / 169.72 kW` is five
    /// significant figures and resolves an error of 0.006%; `2.43 mph /
    /// 3.90 km/h` is three, and resolves only 0.61% — two hundred times
    /// coarser, from the same two decimal places.
    ///
    /// Both directions are considered and the worse is returned, because an
    /// error can go either way and the figure is only protected to the level
    /// that catches both. The printed pair is rarely an exact conversion of
    /// each other, and that standing offset eats margin on one side, so the
    /// two directions are not symmetric.
    #[must_use]
    pub fn resolution(&self) -> Option<f64> {
        if !self.internally_consistent() {
            return None;
        }
        let k = self.conversion.factor();
        let u = half_ulp(self.us_decimals);
        let s = half_ulp(self.si_decimals);
        let base = self.us * k;
        let up = (self.si + s + u * k) / base - 1.0;
        let down = 1.0 - (self.si - s - u * k) / base;
        Some(up.max(down))
    }

    /// The SI value this edition asserts, if its two printed forms agree.
    #[must_use]
    pub fn si_if_consistent(self) -> Option<f64> {
        self.internally_consistent().then_some(self.si)
    }
}

/// What the sources, taken together, say a figure is.
///
/// Two independent checks feed this, and they catch different things:
///
/// 1. **Within one edition** — do its two printed unit forms reconcile? This
///    catches a misread digit, because a corrupted number almost never agrees
///    with its own conversion.
/// 2. **Across editions** — do the editions that pass check 1 agree with each
///    other? This catches what check 1 cannot: a figure taken from the wrong
///    row. A neighbouring row's value reconciles perfectly, because it is a
///    real figure — it is simply the wrong one. Adjacent rows here differ by
///    as little as 0.17%, so nothing within a single edition can separate them.
///
/// ⚠ Neither check says anything about whether the Laboratory *measured*
/// correctly, or mis-printed a figure in every edition. They validate
/// transcription, which is the error these sources actually introduce.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reading {
    /// Every edition that reconciles internally agrees on this SI value.
    Agreed {
        /// The agreed SI value.
        si: f64,
        /// How many editions produced it.
        agreeing: usize,
        /// How many editions were set aside because their own two printed
        /// forms disagreed — a corrupt scan, not a contested measurement.
        rejected: usize,
    },
    /// Editions that each reconcile internally disagree with one another.
    ///
    /// Not present in the current data. It would mean either a genuine change
    /// between printings or a transcription from the wrong row, and both need
    /// a human to look rather than a rule to pick a winner.
    Conflicted {
        /// Lowest SI value any internally-consistent edition gives.
        low: f64,
        /// Highest.
        high: f64,
    },
    /// No edition's two printed forms reconcile, so nothing here is usable.
    Unresolved,
}

impl Reading {
    /// The SI value if the sources agree, otherwise `None`.
    #[must_use]
    pub const fn agreed(self) -> Option<f64> {
        match self {
            Self::Agreed { si, .. } => Some(si),
            Self::Conflicted { .. } | Self::Unresolved => None,
        }
    }

    /// The inclusive span the true SI value lies in, or `None` if nothing is
    /// usable.
    ///
    /// A point for an agreed figure and the disagreement's range for a
    /// conflicted one, so arithmetic over spans propagates doubt rather than
    /// silently picking a side.
    #[must_use]
    pub const fn span(self) -> Option<(f64, f64)> {
        match self {
            Self::Agreed { si, .. } => Some((si, si)),
            Self::Conflicted { low, high } => Some((low, high)),
            Self::Unresolved => None,
        }
    }

    /// How many editions were set aside as internally inconsistent.
    ///
    /// Non-zero means a scan was corrupt and the others outvoted it. Worth
    /// surfacing: it is evidence about that edition's quality, not about the
    /// figure.
    #[must_use]
    pub const fn rejected(self) -> usize {
        match self {
            Self::Agreed { rejected, .. } => rejected,
            Self::Conflicted { .. } | Self::Unresolved => 0,
        }
    }
}

/// One printing of the compilation.
///
/// The same test table is reprinted in every edition from the year after the
/// test onward, so a figure measured once in 2014 has been scanned
/// independently several times. That redundancy is the cross-edition check.
#[derive(Clone, Copy, Debug)]
pub struct Edition {
    /// Cover year of the compilation.
    pub year: u16,
    /// Title as the document titles itself.
    pub title: &'static str,
    /// Where it was retrieved.
    pub url: &'static str,
    /// ISO date of retrieval.
    pub retrieved: &'static str,
}

/// One edition's reading of one figure.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    edition: u16,
    printed: Printed,
}

impl Observation {
    /// Record what one edition prints for a figure.
    #[must_use]
    pub const fn new(edition: u16, printed: Printed) -> Self {
        Self { edition, printed }
    }

    /// Which edition this reading came from.
    #[must_use]
    pub const fn edition(self) -> u16 {
        self.edition
    }

    /// The figure as that edition printed it.
    #[must_use]
    pub const fn printed(self) -> Printed {
        self.printed
    }
}

/// One figure, as read from every edition that carries it legibly.
#[derive(Clone, Copy, Debug)]
pub struct Datum {
    /// Row label, close to the source's own wording.
    pub name: &'static str,
    /// What each edition prints. Never empty.
    pub observations: &'static [Observation],
}

/// How close two editions' SI values must be to count as the same reading.
///
/// They are transcribed literals, so agreement is exact in practice; this is a
/// guard against a last-bit difference, not a tolerance for disagreement. Any
/// real disagreement is orders of magnitude larger — the smallest in this data
/// is 0.18%.
const SAME: f64 = 1e-9;

/// What a set of editions' readings, taken together, say a figure is.
///
/// Split out from [`Datum::read`] so the cross-edition rule can be exercised
/// on observation sets that are not in the dataset — in particular the case it
/// exists to catch, where one edition supplies a real figure from the wrong
/// row. That case cannot be built by editing the table, because the table is
/// what the rule is meant to protect.
#[must_use]
pub fn reconcile(observations: &[Observation]) -> Reading {
    let consistent: Vec<f64> = observations
        .iter()
        .filter_map(|o| o.printed().si_if_consistent())
        .collect();
    let rejected = observations.len() - consistent.len();
    let Some(&first) = consistent.first() else {
        return Reading::Unresolved;
    };
    if consistent.iter().all(|v| (v - first).abs() <= SAME) {
        return Reading::Agreed {
            si: first,
            agreeing: consistent.len(),
            rejected,
        };
    }
    let low = consistent.iter().copied().fold(f64::INFINITY, f64::min);
    let high = consistent.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    Reading::Conflicted { low, high }
}

impl Datum {
    /// What the editions, taken together, say this figure is.
    #[must_use]
    pub fn read(&self) -> Reading {
        reconcile(self.observations)
    }

    /// Editions whose two printed forms disagree with each other.
    ///
    /// These are corrupt scans, outvoted by the rest. Kept because which
    /// edition failed, and on which figure, is evidence about that scan's
    /// quality rather than about the tractor.
    #[must_use]
    pub fn corrupt_editions(&self) -> Vec<u16> {
        self.observations
            .iter()
            .filter(|o| !o.printed().internally_consistent())
            .map(|o| o.edition())
            .collect()
    }
}

/// Something the sources do not contain, recorded so that its absence is a
/// fact in the dataset rather than a gap somebody fills in later from memory.
#[derive(Clone, Copy, Debug)]
pub struct NotInSource {
    /// The quantity that is missing.
    pub what: &'static str,
    /// Why it is missing, and what it would take to obtain it.
    pub why: &'static str,
}

/// One tractor's test results, as printed across several editions.
#[derive(Clone, Copy, Debug)]
pub struct TractorTest {
    /// Make and model as the sources print it.
    pub make_model: &'static str,
    /// Nebraska summary number.
    pub summary_no: u32,
    /// Year the test was conducted.
    pub year_tested: u16,
    /// The editions read, in year order.
    pub editions: &'static [Edition],
    /// The figures.
    pub data: &'static [Datum],
}

impl TractorTest {
    /// Look up one figure by its row label.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<Reading> {
        self.data.iter().find(|d| d.name == name).map(Datum::read)
    }

    /// Row labels for which at least one edition's scan is corrupt.
    #[must_use]
    pub fn with_corrupt_editions(&self) -> Vec<&'static str> {
        self.data
            .iter()
            .filter(|d| !d.corrupt_editions().is_empty())
            .map(|d| d.name)
            .collect()
    }
}

/// What the retrievable sources do not carry.
pub const ABSENT: &[NotInSource] = &[
    NotInSource {
        what: "wheel slip, per load",
        why: "No edition's per-tractor columns carry a slip row; slip appears \
              only in the methodology prose, which states that drawbar runs \
              span the gears from one below the gear at which 15% slip occurs \
              up to 10 mph (16.1 km/h). That bounds the test envelope but gives \
              no slip figure for any load. Per-load slip is printed in the \
              individual ~30-page report, which is served only through a CGI \
              endpoint that refuses automated retrieval.",
    },
    NotInSource {
        what: "the engine speed at which PTO maximum power was measured",
        why: "Not absent so much as un-carried: three editions (2016, 2017, \
              2018) read 2099 rpm and only the 2019 scan reads 2000, so the \
              cross-edition evidence is strong. It is not a Datum because it is \
              printed in one unit with no SI twin, and this crate's figures are \
              dual-unit pairs. Closing it means an observation type for \
              single-unit values, checked across editions only — worth doing \
              when a fuel-rate model needs to divide by it.",
    },
    NotInSource {
        what: "the surface the drawbar tests were run on",
        why: "The methodology section describes the load units, the gear \
              selection and the part-load series, and mentions a test track \
              only in the context of sound measurement. It does not state the \
              drawbar surface. No surface is asserted here: the conversion from \
              a measured drawbar figure to field performance is precisely what \
              the soil model in this chain is for, so assuming the surface \
              would prejudge the term being modelled.",
    },
];

/// Rows the sources print for this tractor that are deliberately not carried.
pub const NOT_TRANSCRIBED: &[NotInSource] = &[NotInSource {
    what: "engine bore and stroke, and displacement",
    why: "Both are printed in dual units and would reconcile like the rest, \
              so they were skipped for relevance rather than difficulty: engine \
              geometry does not enter a drawbar-to-acres chain at any stage. \
              Add them if a combustion model ever needs them — the method is \
              unchanged, the figures are in the same column of the same page.",
}];

/// Editions that carry this tractor but are not read here, and why.
///
/// ⛔ The reason matters more than the exclusion. A scan is excluded when its
/// glyphs cannot be read **without reference to the value being checked** —
/// and resolving an ambiguous glyph by picking whichever reading reconciles
/// would make the reconciliation check circular for that figure. It could
/// never fail again, because it was used to author the input.
pub const EXCLUDED_EDITIONS: &[NotInSource] = &[
    NotInSource {
        what: "the 2015 edition",
        why: "Heavily corrupted in this tractor's column: PTO maximum power \
              renders as `2 1 5. &\\(Ii\"~` and its SI twin as `(J&J.98J0:'l.(}OO`. \
              Reading those requires deciding what they ought to say, which is \
              precisely what the check is supposed to test independently.",
    },
    NotInSource {
        what: "the 2018 edition",
        why: "Mostly legible and it does corroborate the contested figures, but \
              it renders the 50% load fuel economy as `13.4 1(2.6'1)`, which is \
              either 2.61 or 2.64 and cannot be settled from the glyphs alone. \
              Picking the one that reconciles would author the answer into the \
              input. Its 100% load SI figure is also a clear misread (118.04 \
              where the conversion gives 148.04), so it would contribute a \
              rejected observation rather than a deciding one.",
    },
];

/// John Deere 8245R — the calibration target for the acres chain.
///
/// A mechanical-front-wheel-drive row-crop tractor of the class that pulls
/// primary tillage across the Great Plains. The "245" is a model designation,
/// not a measurement: the measured PTO maximum is 215.88 hp / 160.98 kW.
pub const JOHN_DEERE_8245R: TractorTest = TractorTest {
    make_model: "John Deere 8245R Dsl",
    summary_no: 963,
    year_tested: 2014,
    editions: &[
        Edition {
            year: 2016,
            title: "Nebraska and OECD Tractor Test Data for 2016, \
                    Nebraska Tractor Test Laboratory, MP 37 TTL",
            url: "https://govdocs.nebraska.gov/epubs/U2060/S001-2016.pdf",
            retrieved: "2026-09-18",
        },
        Edition {
            year: 2017,
            title: "Nebraska and OECD Tractor Test Data for 2017, \
                    Nebraska Tractor Test Laboratory, MP 37 TTL",
            url: "https://govdocs.nebraska.gov/epubs/U2060/S001-2017.pdf",
            retrieved: "2026-09-18",
        },
        Edition {
            year: 2019,
            title: "Nebraska and OECD Tractor Test Data for 2019, \
                    Nebraska Tractor Test Laboratory, MP 37 TTL",
            url: "https://govdocs.nebraska.gov/epubs/U2060/S001-2019.pdf",
            retrieved: "2026-09-18",
        },
    ],
    data: &[
        Datum {
            name: "pto max power",
            observations: &[
                Observation::new(2016, Printed::new(215.88, 2, 160.98, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(215.88, 2, 160.98, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(215.88, 2, 160.98, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "pto max power fuel rate",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(11.84, 2, 44.81, 2, Conversion::GalPerHrToLPerH),
                ),
                Observation::new(
                    2017,
                    Printed::new(11.84, 2, 44.81, 2, Conversion::GalPerHrToLPerH),
                ),
                Observation::new(
                    2019,
                    Printed::new(11.84, 2, 44.81, 2, Conversion::GalPerHrToLPerH),
                ),
            ],
        },
        Datum {
            name: "pto max power fuel economy",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(18.24, 2, 3.59, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2017,
                    Printed::new(18.24, 2, 3.59, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2019,
                    Printed::new(18.24, 2, 3.59, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
            ],
        },
        Datum {
            name: "pto power at standard 1000 rpm",
            observations: &[
                Observation::new(2016, Printed::new(235.92, 2, 175.93, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(235.92, 2, 175.93, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(235.92, 2, 175.93, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "pto 1000 rpm fuel rate",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(12.52, 2, 47.40, 2, Conversion::GalPerHrToLPerH),
                ),
                Observation::new(
                    2017,
                    Printed::new(12.52, 2, 47.40, 2, Conversion::GalPerHrToLPerH),
                ),
                Observation::new(
                    2019,
                    Printed::new(12.52, 2, 47.40, 2, Conversion::GalPerHrToLPerH),
                ),
            ],
        },
        Datum {
            name: "pto 1000 rpm fuel economy",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(18.84, 2, 3.71, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2017,
                    Printed::new(18.84, 2, 3.71, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2019,
                    Printed::new(18.84, 2, 3.71, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
            ],
        },
        Datum {
            name: "weight as tested",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(25380.0, 0, 11512.0, 0, Conversion::LbToKg),
                ),
                Observation::new(
                    2017,
                    Printed::new(25380.0, 0, 11512.0, 0, Conversion::LbToKg),
                ),
                Observation::new(
                    2019,
                    Printed::new(25380.0, 0, 11512.0, 0, Conversion::LbToKg),
                ),
            ],
        },
        Datum {
            name: "drawbar max power short term",
            observations: &[
                Observation::new(2016, Printed::new(227.60, 2, 169.72, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(227.60, 2, 169.72, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(227.60, 2, 169.72, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "drawbar max power speed",
            observations: &[
                Observation::new(2016, Printed::new(6.80, 2, 10.94, 2, Conversion::MphToKmh)),
                Observation::new(2017, Printed::new(6.80, 2, 10.94, 2, Conversion::MphToKmh)),
                Observation::new(2019, Printed::new(6.80, 2, 10.94, 2, Conversion::MphToKmh)),
            ],
        },
        Datum {
            name: "drawbar 100pct load power",
            observations: &[
                Observation::new(2016, Printed::new(198.53, 2, 148.04, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(198.53, 2, 148.04, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(198.53, 2, 148.04, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "drawbar 100pct load speed",
            observations: &[
                Observation::new(2016, Printed::new(4.68, 2, 7.53, 2, Conversion::MphToKmh)),
                Observation::new(2017, Printed::new(4.68, 2, 7.53, 2, Conversion::MphToKmh)),
                Observation::new(2019, Printed::new(4.68, 2, 7.53, 2, Conversion::MphToKmh)),
            ],
        },
        Datum {
            name: "drawbar 100pct load fuel economy",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(16.82, 2, 3.31, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2017,
                    Printed::new(16.82, 2, 3.31, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2019,
                    Printed::new(16.82, 2, 3.31, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
            ],
        },
        Datum {
            name: "drawbar 75pct load power",
            observations: &[
                Observation::new(2016, Printed::new(154.47, 2, 115.19, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(154.47, 2, 115.19, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(154.17, 2, 115.19, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "drawbar 75pct load speed",
            observations: &[
                Observation::new(2016, Printed::new(4.85, 2, 7.81, 2, Conversion::MphToKmh)),
                Observation::new(2017, Printed::new(4.85, 2, 7.81, 2, Conversion::MphToKmh)),
                Observation::new(2019, Printed::new(4.85, 2, 7.81, 2, Conversion::MphToKmh)),
            ],
        },
        Datum {
            name: "drawbar 75pct load fuel economy",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(15.72, 2, 3.10, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2017,
                    Printed::new(15.72, 2, 3.10, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2019,
                    Printed::new(15.72, 2, 3.10, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
            ],
        },
        Datum {
            name: "drawbar 50pct load power",
            observations: &[
                Observation::new(2016, Printed::new(104.27, 2, 77.75, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(104.27, 2, 77.75, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(104.27, 2, 77.75, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "drawbar 50pct load speed",
            observations: &[
                Observation::new(2016, Printed::new(4.91, 2, 7.90, 2, Conversion::MphToKmh)),
                Observation::new(2017, Printed::new(4.91, 2, 7.90, 2, Conversion::MphToKmh)),
                Observation::new(2019, Printed::new(4.91, 2, 7.90, 2, Conversion::MphToKmh)),
            ],
        },
        Datum {
            name: "drawbar 50pct load fuel economy",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(13.41, 2, 2.64, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2017,
                    Printed::new(13.41, 2, 2.64, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2019,
                    Printed::new(13.41, 2, 2.61, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
            ],
        },
        Datum {
            name: "drawbar 50pct load reduced rpm power",
            observations: &[
                Observation::new(2016, Printed::new(104.45, 2, 77.88, 2, Conversion::HpToKw)),
                Observation::new(2017, Printed::new(104.45, 2, 77.88, 2, Conversion::HpToKw)),
                Observation::new(2019, Printed::new(104.45, 2, 77.88, 2, Conversion::HpToKw)),
            ],
        },
        Datum {
            name: "drawbar 50pct load reduced rpm speed",
            observations: &[
                Observation::new(2016, Printed::new(4.95, 2, 7.97, 2, Conversion::MphToKmh)),
                Observation::new(2017, Printed::new(4.95, 2, 7.97, 2, Conversion::MphToKmh)),
                Observation::new(2019, Printed::new(4.95, 2, 7.97, 2, Conversion::MphToKmh)),
            ],
        },
        Datum {
            name: "drawbar 50pct load reduced rpm fuel economy",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(16.66, 2, 3.28, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2017,
                    Printed::new(16.66, 2, 3.28, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
                Observation::new(
                    2019,
                    Printed::new(16.66, 2, 3.28, 2, Conversion::HpHrPerGalToKwhPerL),
                ),
            ],
        },
        Datum {
            name: "drawbar max pull",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(24702.0, 0, 109.88, 2, Conversion::LbfToKn),
                ),
                Observation::new(
                    2017,
                    Printed::new(24702.0, 0, 109.88, 2, Conversion::LbfToKn),
                ),
                Observation::new(
                    2019,
                    Printed::new(24702.0, 0, 109.68, 2, Conversion::LbfToKn),
                ),
            ],
        },
        Datum {
            name: "drawbar max pull speed",
            observations: &[
                Observation::new(2016, Printed::new(2.43, 2, 3.90, 2, Conversion::MphToKmh)),
                Observation::new(2017, Printed::new(2.43, 2, 3.90, 2, Conversion::MphToKmh)),
                Observation::new(2019, Printed::new(2.43, 2, 3.90, 2, Conversion::MphToKmh)),
            ],
        },
        Datum {
            name: "three point lift at 24in behind hitch",
            observations: &[
                Observation::new(2016, Printed::new(14274.0, 0, 63.5, 1, Conversion::LbfToKn)),
                Observation::new(2017, Printed::new(14274.0, 0, 63.5, 1, Conversion::LbfToKn)),
                Observation::new(2019, Printed::new(14274.0, 0, 63.5, 1, Conversion::LbfToKn)),
            ],
        },
        Datum {
            name: "hydraulic flow",
            observations: &[
                Observation::new(
                    2016,
                    Printed::new(60.6, 1, 229.3, 1, Conversion::GalPerMinToLPerMin),
                ),
                Observation::new(
                    2017,
                    Printed::new(60.6, 1, 229.3, 1, Conversion::GalPerMinToLPerMin),
                ),
                Observation::new(
                    2019,
                    Printed::new(60.6, 1, 229.3, 1, Conversion::GalPerMinToLPerMin),
                ),
            ],
        },
    ],
};

/// Figures where one edition's scan is corrupt, and which edition it is.
///
/// Every entry is an observation whose own two printed unit forms disagree, so
/// it is set aside and the remaining editions decide. Pinned as a roster and
/// checked in both directions: a scan that starts failing, or stops, changes
/// this list rather than passing quietly.
///
/// ⚠ All three are the **2019** edition, and all three are its **SI** column —
/// `115.19` printed against a US figure of `154.17` where the other editions
/// print `154.47`, and `2.61` and `109.68` where the others give `2.64` and
/// `109.88`. That pattern is evidence about one scan, which is exactly what a
/// single-source dataset cannot tell you.
pub const CORRUPT_SCANS: &[(&str, u16)] = &[
    ("drawbar 75pct load power", 2019),
    ("drawbar 50pct load fuel economy", 2019),
    ("drawbar max pull", 2019),
];

/// Figures whose *within-edition* check is too coarse to catch an error the
/// size this data actually contains.
///
/// Every observation of these resolves worse than [`CATCHABLE_ERROR`], so for
/// them the cross-edition agreement is doing the work rather than the
/// unit-conversion check.
///
/// ⚠ The membership is one-directional. **Every weakly-checked figure is a
/// speed or a fuel economy** — printed at three significant figures where
/// powers and masses get five. The converse is false: 3 of the 6 speeds and 1
/// of the 6 fuel economies resolve better than 0.2% and are not listed.
pub const WEAKLY_CHECKED_8245R: &[&str] = &[
    "pto max power fuel economy",
    "drawbar 100pct load fuel economy",
    "drawbar 75pct load speed",
    "drawbar 75pct load fuel economy",
    "drawbar 50pct load fuel economy",
    "drawbar 50pct load reduced rpm speed",
    "drawbar 50pct load reduced rpm fuel economy",
    "drawbar max pull speed",
];

/// The relative error [`WEAKLY_CHECKED_8245R`] is defined against.
///
/// Set to the size of the real scan errors this dataset contains: the 2019
/// edition's `154.17` against the other editions' `154.47` is 0.19%, and its
/// `109.68` against `109.88` is 0.18%.
pub const CATCHABLE_ERROR: f64 = 0.002;
