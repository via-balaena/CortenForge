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
//! annual compilation, and the copy of it that can be fetched is a **scan**:
//! the text layer is OCR, and OCR of a numeric table is exactly the kind of
//! input that produces a wrong number which looks right.
//!
//! The compilation prints every figure **twice** — once in US customary units
//! and once in SI. That redundancy is a checksum, and this crate is built
//! around it: a figure is trustworthy only if its two printed forms reconcile
//! under the unit conversion, allowing for the rounding implied by how many
//! digits each was printed to. See [`Printed::read`].
//!
//! Three figures in the 8245R column do **not** reconcile. They are not
//! guessed at and not dropped — they are carried as [`Reading::Disputed`],
//! which cannot be read as a single number without the caller acknowledging
//! the disagreement.
//!
//! # What this crate deliberately does NOT contain
//!
//! See [`ABSENT`]. The compilation carries no wheel-slip column, does not state
//! the surface the drawbar tests were run on, and is ambiguous about the engine
//! speed at PTO maximum power. All three are recorded as absences with their
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
    /// forms disagree. The validated value comes from [`Printed::read`], which
    /// cannot hand back a number for a disputed figure without saying so.
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

    /// Reconcile the two printed forms.
    ///
    /// The printed US figure stands for a true value within its own rounding
    /// interval; converting that whole interval gives the range of SI values
    /// consistent with it. If that range overlaps the printed SI figure's own
    /// rounding interval, the two agree.
    #[must_use]
    pub fn read(&self) -> Reading {
        let k = self.conversion.factor();
        let u = half_ulp(self.us_decimals);
        let s = half_ulp(self.si_decimals);
        let (lo, hi) = ((self.us - u) * k, (self.us + u) * k);
        if hi < self.si - s || lo > self.si + s {
            Reading::Disputed {
                from_us: self.us * k,
                as_printed: self.si,
            }
        } else {
            Reading::Agreed(self.si)
        }
    }

    /// The smallest relative error in the US figure this check is guaranteed
    /// to catch, or `None` if the figure is already disputed.
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
        self.read().agreed()?;
        let k = self.conversion.factor();
        let u = half_ulp(self.us_decimals);
        let s = half_ulp(self.si_decimals);
        let base = self.us * k;
        let up = (self.si + s + u * k) / base - 1.0;
        let down = 1.0 - (self.si - s - u * k) / base;
        Some(up.max(down))
    }
}

/// What a [`Printed`] figure can be trusted to say.
///
/// A figure whose two printed forms disagree is still *usable* — the
/// disagreement is small and its bounds are known — but there is no path to a
/// **validated** value that does not go through this enum, so a caller cannot
/// treat a disputed figure as settled by accident.
///
/// ⚠ The raw printed figures remain reachable through
/// [`Printed::as_printed_us`] and [`Printed::as_printed_si`], because
/// transcription provenance is part of what this crate is for. They are named
/// to announce what they are: what is on the page, not what has been checked.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reading {
    /// The two printed forms reconcile. Carries the SI value.
    Agreed(f64),
    /// The two printed forms disagree: converting the printed US value does
    /// not land within the printed SI value's rounding interval.
    ///
    /// Almost always a scanning error in one of the two, but which one cannot
    /// be settled without the original report, so both candidates are kept.
    Disputed {
        /// SI value implied by the printed US figure.
        from_us: f64,
        /// SI value as printed.
        as_printed: f64,
    },
}

impl Reading {
    /// The SI value if the two printed forms agree, otherwise `None`.
    #[must_use]
    pub const fn agreed(self) -> Option<f64> {
        match self {
            Self::Agreed(v) => Some(v),
            Self::Disputed { .. } => None,
        }
    }

    /// The inclusive span the true SI value lies in, disputed or not.
    ///
    /// For an agreed figure this is a point. For a disputed one it spans both
    /// candidates, so arithmetic over it propagates the disagreement instead
    /// of silently picking a side.
    #[must_use]
    pub const fn span(self) -> (f64, f64) {
        match self {
            Self::Agreed(v) => (v, v),
            Self::Disputed {
                from_us,
                as_printed,
            } => (from_us.min(as_printed), from_us.max(as_printed)),
        }
    }
}

/// Where a figure came from, precisely enough to go and look.
///
/// Carried on the test rather than on each figure because every figure in a
/// test comes off the same page of the same document. A number with no
/// producer is not a measurement, so there is no way to build a [`TractorTest`]
/// without one.
#[derive(Clone, Copy, Debug)]
pub struct Source {
    /// Title of the document, as it titles itself.
    pub document: &'static str,
    /// Where the document was retrieved from.
    pub url: &'static str,
    /// ISO date of retrieval.
    pub retrieved: &'static str,
    /// How to find the figures within the document.
    pub locator: &'static str,
    /// How the text was obtained, and what that costs in trustworthiness.
    pub extraction: &'static str,
}

/// One named figure from a test.
#[derive(Clone, Copy, Debug)]
pub struct Datum {
    /// Row label, close to the source's own wording.
    pub name: &'static str,
    /// The figure, in both printed unit systems.
    pub printed: Printed,
}

/// One tractor's test results.
#[derive(Clone, Copy, Debug)]
pub struct TractorTest {
    /// Make and model as the source prints it.
    pub make_model: &'static str,
    /// Nebraska summary number.
    pub summary_no: u32,
    /// Year the test was conducted.
    pub year_tested: u16,
    /// Provenance for every figure below.
    pub source: Source,
    /// The figures.
    pub data: &'static [Datum],
}

impl TractorTest {
    /// Look up one figure by its row label.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<Reading> {
        self.data
            .iter()
            .find(|d| d.name == name)
            .map(|d| d.printed.read())
    }

    /// Every row label whose two printed forms disagree, in declaration order.
    #[must_use]
    pub fn disputed(&self) -> Vec<&'static str> {
        self.data
            .iter()
            .filter(|d| d.printed.read().agreed().is_none())
            .map(|d| d.name)
            .collect()
    }
}

/// Something the source does not contain, recorded so that its absence is a
/// fact in the dataset rather than a gap somebody fills in later from memory.
#[derive(Clone, Copy, Debug)]
pub struct NotInSource {
    /// The quantity that is missing.
    pub what: &'static str,
    /// Why it is missing, and what it would take to obtain it.
    pub why: &'static str,
}

/// What the retrievable source does not carry.
///
/// ⚠ MISSION names *"Nebraska Tractor Test drawbar **and slip**"* as the ground
/// truth for the tractor half. Half of that oracle is not in the reachable
/// document. Recording that here is the difference between a known gap and a
/// number someone invents later.
pub const ABSENT: &[NotInSource] = &[
    NotInSource {
        what: "wheel slip, per load",
        why: "The annual compilation's per-tractor columns carry no slip row; \
              slip appears only in the methodology prose, which states that \
              drawbar runs span the gears from one below the gear at which 15% \
              slip occurs up to 10 mph (16.1 km/h). That bounds the test \
              envelope but gives no slip figure for any load. Per-load slip is \
              printed in the individual ~30-page report, which is served only \
              through a CGI endpoint that refuses automated retrieval.",
    },
    NotInSource {
        what: "the engine speed at which PTO maximum power was measured",
        why: "The row is labelled max power at RATED engine speed, and rated \
              RPM for this tractor is printed as 2100 — but the figure in the \
              8245R column reads as 2000, where both neighbouring columns read \
              2100. The scan does not settle which, and the dual-unit check \
              cannot help: an engine speed is printed once, in rpm, with no SI \
              twin to reconcile against. Recorded rather than picked, because a \
              fuel-rate model divides by this number.",
    },
    NotInSource {
        what: "the surface the drawbar tests were run on",
        why: "The retrieved methodology section describes the load units, the \
              gear selection and the part-load series, and mentions a test \
              track only in the context of sound measurement. It does not \
              state the drawbar surface. No surface is asserted here: the \
              conversion from a measured drawbar figure to field performance \
              is precisely what the soil model in this chain is for, so \
              assuming the surface would prejudge the term being modelled.",
    },
];

/// Rows the source prints for this tractor that are deliberately not carried.
///
/// The 25 figures in [`JOHN_DEERE_8245R`] are not the whole column, and
/// `the_test_identifies_itself` pins that count — so what the count excludes
/// has to be written down, or a reader cannot tell a deliberate omission from
/// a transcription that stopped early.
pub const NOT_TRANSCRIBED: &[NotInSource] = &[NotInSource {
    what: "engine bore and stroke, and displacement",
    why: "Both are printed in dual units and would reconcile like the rest, \
              so they were skipped for relevance rather than difficulty: engine \
              geometry does not enter a drawbar-to-acres chain at any stage. \
              Add them if a combustion model ever needs them — the method is \
              unchanged, the figures are in the same column of the same page.",
}];

/// John Deere 8245R — the calibration target for the acres chain.
///
/// A ~245 hp mechanical-front-wheel-drive row-crop tractor: the class that
/// actually pulls primary tillage across the upper Midwest, and the class
/// where on-farm hydrogen production is arguable rather than absurd.
pub const JOHN_DEERE_8245R: TractorTest = TractorTest {
    make_model: "John Deere 8245R Dsl",
    summary_no: 963,
    year_tested: 2014,
    source: Source {
        document: "Nebraska and OECD Tractor Test Data for 2019 \
                   (containing test data through December 2018), \
                   Nebraska Tractor Test Laboratory, MP 37 TTL",
        url: "https://govdocs.nebraska.gov/epubs/U2060/S001-2019.pdf",
        retrieved: "2026-09-18",
        locator: "John Deere section, three-column page carrying \
                  7310R / 7310R / 8245R; the 8245R is the right-hand column",
        extraction: "pdftotext -layout over a scanned page. The text layer is \
                     OCR and is visibly lossy in the prose; every figure below \
                     is therefore carried in both printed unit systems and \
                     reconciled against its own conversion.",
    },
    data: &[
        // ---- PTO -------------------------------------------------------
        Datum {
            name: "pto max power",
            printed: Printed::new(215.88, 2, 160.98, 2, Conversion::HpToKw),
        },
        Datum {
            name: "pto max power fuel rate",
            printed: Printed::new(11.84, 2, 44.81, 2, Conversion::GalPerHrToLPerH),
        },
        Datum {
            name: "pto max power fuel economy",
            printed: Printed::new(18.24, 2, 3.59, 2, Conversion::HpHrPerGalToKwhPerL),
        },
        Datum {
            name: "pto power at standard 1000 rpm",
            printed: Printed::new(235.92, 2, 175.93, 2, Conversion::HpToKw),
        },
        Datum {
            name: "pto 1000 rpm fuel rate",
            printed: Printed::new(12.52, 2, 47.40, 2, Conversion::GalPerHrToLPerH),
        },
        Datum {
            name: "pto 1000 rpm fuel economy",
            printed: Printed::new(18.84, 2, 3.71, 2, Conversion::HpHrPerGalToKwhPerL),
        },
        // ---- mass ------------------------------------------------------
        Datum {
            name: "weight as tested",
            printed: Printed::new(25380.0, 0, 11512.0, 0, Conversion::LbToKg),
        },
        // ---- drawbar ---------------------------------------------------
        Datum {
            name: "drawbar max power short term",
            printed: Printed::new(227.60, 2, 169.72, 2, Conversion::HpToKw),
        },
        Datum {
            name: "drawbar max power speed",
            printed: Printed::new(6.80, 2, 10.94, 2, Conversion::MphToKmh),
        },
        Datum {
            name: "drawbar 100pct load power",
            printed: Printed::new(198.53, 2, 148.04, 2, Conversion::HpToKw),
        },
        Datum {
            name: "drawbar 100pct load speed",
            printed: Printed::new(4.68, 2, 7.53, 2, Conversion::MphToKmh),
        },
        Datum {
            name: "drawbar 100pct load fuel economy",
            printed: Printed::new(16.82, 2, 3.31, 2, Conversion::HpHrPerGalToKwhPerL),
        },
        Datum {
            name: "drawbar 75pct load power",
            printed: Printed::new(154.17, 2, 115.19, 2, Conversion::HpToKw),
        },
        Datum {
            name: "drawbar 75pct load speed",
            printed: Printed::new(4.85, 2, 7.81, 2, Conversion::MphToKmh),
        },
        Datum {
            name: "drawbar 75pct load fuel economy",
            printed: Printed::new(15.72, 2, 3.10, 2, Conversion::HpHrPerGalToKwhPerL),
        },
        Datum {
            name: "drawbar 50pct load power",
            printed: Printed::new(104.27, 2, 77.75, 2, Conversion::HpToKw),
        },
        Datum {
            name: "drawbar 50pct load speed",
            printed: Printed::new(4.91, 2, 7.90, 2, Conversion::MphToKmh),
        },
        Datum {
            name: "drawbar 50pct load fuel economy",
            printed: Printed::new(13.41, 2, 2.61, 2, Conversion::HpHrPerGalToKwhPerL),
        },
        Datum {
            name: "drawbar 50pct load reduced rpm power",
            printed: Printed::new(104.45, 2, 77.88, 2, Conversion::HpToKw),
        },
        Datum {
            name: "drawbar 50pct load reduced rpm speed",
            printed: Printed::new(4.95, 2, 7.97, 2, Conversion::MphToKmh),
        },
        Datum {
            name: "drawbar 50pct load reduced rpm fuel economy",
            printed: Printed::new(16.66, 2, 3.28, 2, Conversion::HpHrPerGalToKwhPerL),
        },
        Datum {
            name: "drawbar max pull",
            printed: Printed::new(24702.0, 0, 109.68, 2, Conversion::LbfToKn),
        },
        Datum {
            name: "drawbar max pull speed",
            printed: Printed::new(2.43, 2, 3.90, 2, Conversion::MphToKmh),
        },
        // ---- hitch and hydraulics --------------------------------------
        Datum {
            name: "three point lift at 24in behind hitch",
            printed: Printed::new(14274.0, 0, 63.5, 1, Conversion::LbfToKn),
        },
        Datum {
            name: "hydraulic flow",
            printed: Printed::new(60.6, 1, 229.3, 1, Conversion::GalPerMinToLPerMin),
        },
    ],
};

/// The figures in [`JOHN_DEERE_8245R`] whose two printed forms do not agree.
///
/// Pinned as a roster so that the set is checked in both directions: a figure
/// that starts reconciling, or stops, fails the test rather than passing
/// quietly. Each entry is a genuine disagreement in the scan, not a tolerance
/// that wants loosening — see `disputed_entries_disagree_by_a_digit_not_a_rounding`.
pub const DISPUTED_8245R: &[&str] = &[
    "drawbar 75pct load power",
    "drawbar 50pct load fuel economy",
    "drawbar max pull",
];

/// Figures whose two printed forms agree, but not tightly enough to catch an
/// error the size of the ones this scan actually contains.
///
/// Reconciliation is not a binary: it resolves an error only down to the
/// precision the source printed. Every entry here has a
/// [`Printed::resolution`] coarser than 0.2%, the size of the real
/// disagreements in [`DISPUTED_8245R`]. An OCR error of that size in one of
/// these figures would pass unnoticed.
///
/// ⚠ The membership is not arbitrary, but it is one-directional, and an
/// earlier draft of this comment had it backwards. **Every weakly-checked
/// figure is a speed or a fuel economy** — those are printed at three
/// significant figures where the powers and masses get five. The converse is
/// false: 3 of the 6 speeds and 1 of the 5 undisputed fuel economies resolve
/// better than 0.2% and are *not* listed here. `only_low_significance_figures_are_weakly_checked`
/// tests the direction that holds, and nothing tests the one that does not,
/// because it is not true.
///
/// ⇒ Lean the chain on the power, mass and pull figures; treat these seven as
/// corroborating, not load-bearing.
///
/// The counterpart worth stating: `drawbar max pull` resolves to 0.008%, so
/// its presence in [`DISPUTED_8245R`] is a real disagreement in the scan and
/// not an artifact of coarse printing.
pub const WEAKLY_CHECKED_8245R: &[&str] = &[
    "pto max power fuel economy",
    "drawbar 100pct load fuel economy",
    "drawbar 75pct load speed",
    "drawbar 75pct load fuel economy",
    "drawbar 50pct load reduced rpm speed",
    "drawbar 50pct load reduced rpm fuel economy",
    "drawbar max pull speed",
];

/// The relative error [`WEAKLY_CHECKED_8245R`] is defined against.
///
/// Set to the size of the disagreements actually found in this scan — 0.19%
/// for `154.17` against an implied `154.47`, 0.18% for `24702 lbf` against an
/// implied `24657` — because a check that cannot resolve that would not have
/// found either of them.
pub const CATCHABLE_ERROR: f64 = 0.002;
