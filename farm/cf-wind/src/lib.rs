//! One farm site's wind year, and the energy a named turbine would take from it.
//!
//! This is the first stage of the acres-per-season chain: MISSION asks how many
//! acres per season one farm can run on its own wind-derived hydrogen, and
//! everything downstream starts from how much energy the wind actually carries
//! at one real place.
//!
//! # ⛔⛔ The error this crate exists to make impossible
//!
//! **Wind power follows the mean of the cube, not the cube of the mean.** At
//! this site the two differ by a factor of **1.67** — estimating from the
//! average wind speed understates the available power by 40%. It is the most
//! common mistake in wind resource estimation, it is invisible once it is in a
//! spreadsheet, and it would propagate through every remaining stage of the
//! chain. [`WindYear::mean_speed`] and [`WindYear::mean_cube_speed`] are both
//! public precisely so the difference is in front of you, and
//! `the_cube_of_the_mean_is_not_the_mean_of_the_cube` pins it.
//!
//! # ⚠ What these numbers are, and are not
//!
//! MISSION says *"that farm's measured wind year"*. **These are not
//! measurements.** The WIND Toolkit is a WRF reanalysis on a 2 km grid — a
//! physical model reconstructing the atmosphere, not an anemometer on this
//! farm. See [`NOT_MEASURED_HERE`].
//!
//! ⚠ It is not unchecked, though. [`CARRINGTON_AIRPORT`] compares it against the
//! nearest observing station — 1.96 km away, 6 m different in elevation — over
//! 7128 overlapping hours of 2012: the implied 10 m → 100 m shear exponent is
//! 0.209 and the hourly correlation is 0.61. The resource is corroborated; any
//! individual hour is not.
//!
//! ⚠⚠ That also makes this crate's uncertainty a **different kind** from
//! `cf-nebraska`'s. There, two printed copies of one number could disagree and
//! the question was which scan was corrupt — transcription error, resolvable by
//! a third source. Here the number is not in dispute: the model says what it
//! says. The uncertainty is *model-vs-world*, and no amount of re-reading
//! settles it. The two must not share a type just because both are "a number
//! with doubt attached".

/// Where the series came from, precisely enough to fetch it again.
#[derive(Clone, Copy, Debug)]
pub struct Provenance {
    /// Public S3 bucket hosting the WIND Toolkit.
    pub bucket: &'static str,
    /// Object key within the bucket.
    pub key: &'static str,
    /// Dataset read from that HDF5 file.
    pub dataset: &'static str,
    /// Column index into the dataset's spatial axis.
    pub site_index: usize,
    /// Divisor taking the stored integers to m/s, as the dataset's own
    /// `scale_factor` attribute states it.
    pub scale_factor: f64,
    /// Value the dataset uses for missing data, from its `fill_value` attribute.
    pub fill_value: u16,
    /// ISO date the series was pulled.
    pub retrieved: &'static str,
    /// SHA-256 of `data/`'s bytes.
    ///
    /// ⚠ Checked by `shasum -a 256`, **not** by a unit test — this crate has no
    /// dependencies and adding the workspace's first hash crate to verify one
    /// file would cost more than it buys. The test-time drift guard is
    /// [`WindYear::checksum`], which is cheap and needs nothing.
    pub sha256: &'static str,
    /// How to rebuild the file from the bucket.
    pub reproduce: &'static str,
}

/// The grid point, as the source file identifies it.
///
/// Every field here is read out of the file's own `meta` and `coordinates`
/// datasets rather than asserted: the WIND Toolkit states which state and
/// county each index falls in, so the site's identity is the source's claim,
/// not mine.
///
/// ⚠ What that does not establish: nothing here checks the WIND Toolkit's
/// geolocation is itself correct.
#[derive(Clone, Copy, Debug)]
pub struct Site {
    /// Index into the dataset's spatial axis.
    pub index: usize,
    /// Grid-point latitude, degrees north.
    pub latitude: f64,
    /// Grid-point longitude, degrees east (negative west).
    pub longitude: f64,
    /// US state, as the file's `meta` records it.
    pub state: &'static str,
    /// County, as the file's `meta` records it.
    pub county: &'static str,
    /// Elevation above sea level, metres, from `meta`.
    pub elevation_m: i32,
    /// Nearest named town, and how far the grid point sits from it.
    pub nearest_town: &'static str,
    /// Distance from `nearest_town`, kilometres.
    pub km_from_town: f64,
    /// Height above ground the wind speeds apply to, metres.
    pub measurement_height_m: f64,
}

/// Something the source is not, recorded so nobody later assumes it is.
#[derive(Clone, Copy, Debug)]
pub struct Caveat {
    /// The thing that might be assumed.
    pub what: &'static str,
    /// Why it is not so, and what would close the gap.
    pub why: &'static str,
}

/// ⚠ The gap between what MISSION asks for and what this crate has.
pub const NOT_MEASURED_HERE: &[Caveat] = &[
    Caveat {
        what: "these are modelled wind speeds, not measurements at this farm",
        why: "The WIND Toolkit is a Weather Research and Forecasting reanalysis \
              on a 2 km grid; no anemometer stood at this grid point, and \
              MISSION asks for a farm's MEASURED wind year. It is not unchecked, \
              though: see CARRINGTON_AIRPORT, which compares it against the \
              nearest observing station rather than calling it validated in the \
              abstract. Closing the gap entirely still means measurement on the \
              farm itself.",
    },
    Caveat {
        what: "one year is not a wind resource",
        why: "2012 is a single realisation. Inter-annual variability in the \
              northern Plains is commonly several percent of mean wind power \
              density, so an annual energy figure from one year carries that \
              spread before any other uncertainty is added. The Toolkit offers \
              2007-2013; using one year is a deliberate starting point, not a \
              claim that years are alike.",
    },
    Caveat {
        what: "the hub height is the dataset's height, not a turbine's",
        why: "Speeds are the WIND Toolkit's 100 m layer. A turbine whose hub is \
              not at 100 m needs shear extrapolation, which is a modelling step \
              with its own error and is deliberately not done here — doing it \
              silently would bury an assumption inside what looks like data.",
    },
];

/// The wind speeds, stored exactly as the source encodes them.
///
/// Scaled 16-bit integers, little-endian, no transformation at rest: whatever
/// is wrong with them is the source's, not this crate's.
const SAMPLES: &[u8] = include_bytes!("../data/windspeed_100m_2012_foster_nd_u16le.bin");

/// Seconds between consecutive samples.
const INTERVAL_S: f64 = 300.0;

/// A year of wind speeds at one grid point.
#[derive(Clone, Copy, Debug)]
pub struct WindYear {
    /// Calendar year the samples cover.
    pub year: u16,
    /// Where they were taken.
    pub site: Site,
    /// Where they came from.
    pub provenance: Provenance,
    raw: &'static [u8],
}

impl WindYear {
    /// Build a wind year from scaled little-endian `u16` samples.
    ///
    /// Public because the wind year is a **seam**: the chain has to be able to
    /// run on another site, another year, or a deliberately contrived series,
    /// or the sweep cannot say how much this stage moves the answer. A stage
    /// that can only be run on the one series shipped with it is not seamed,
    /// it is hard-coded.
    #[must_use]
    pub const fn from_scaled_le_bytes(
        year: u16,
        site: Site,
        provenance: Provenance,
        raw: &'static [u8],
    ) -> Self {
        Self {
            year,
            site,
            provenance,
            raw,
        }
    }

    /// Number of samples.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.raw.len() / 2
    }

    /// Whether the series is empty. Never true for the shipped year.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Seconds each sample represents.
    #[must_use]
    pub const fn interval_seconds(&self) -> f64 {
        INTERVAL_S
    }

    /// Wind speed at one sample, m/s.
    ///
    /// Returns `None` past the end, or if the source marked the sample missing.
    #[must_use]
    pub fn speed(&self, i: usize) -> Option<f64> {
        // ⚠ checked: `i * 2` overflows for a large index and panics in debug,
        // which is how a lookup that promises `None` past the end became a
        // crash. Found by the gate, not by review.
        let lo = i.checked_mul(2)?;
        let bytes = [*self.raw.get(lo)?, *self.raw.get(lo.checked_add(1)?)?];
        let v = u16::from_le_bytes(bytes);
        (v != self.provenance.fill_value).then(|| f64::from(v) / self.provenance.scale_factor)
    }

    /// Every speed in order, m/s, skipping any the source marked missing.
    pub fn speeds(&self) -> impl Iterator<Item = f64> + '_ {
        (0..self.len()).filter_map(|i| self.speed(i))
    }

    /// Arithmetic mean wind speed, m/s.
    ///
    /// ⛔ **Not** a basis for estimating power. See
    /// [`WindYear::mean_cube_speed`].
    #[must_use]
    pub fn mean_speed(&self) -> f64 {
        let (n, s) = self.speeds().fold((0.0, 0.0), |(n, s), v| (n + 1.0, s + v));
        if n == 0.0 { 0.0 } else { s / n }
    }

    /// Mean of the **cube** of wind speed, m³/s³ — what power is proportional to.
    ///
    /// ⛔⛔ This is not `mean_speed().powi(3)`, and the gap is not small: at
    /// this site the ratio is 1.67, so estimating from the average speed
    /// understates available power by 40%. Jensen's inequality guarantees the
    /// direction — the cube is convex, so the mean of the cube is always at
    /// least the cube of the mean — but the magnitude is a property of this
    /// site's distribution and has to be measured, not assumed.
    #[must_use]
    pub fn mean_cube_speed(&self) -> f64 {
        let (n, s) = self
            .speeds()
            .fold((0.0, 0.0), |(n, s), v| (n + 1.0, s + v * v * v));
        if n == 0.0 { 0.0 } else { s / n }
    }

    /// FNV-1a over the stored bytes — the drift guard.
    ///
    /// Detects a changed checked-in file. It is not a cryptographic claim and
    /// is not what ties the file to the bucket; [`Provenance::sha256`] does
    /// that, checked with `shasum`.
    #[must_use]
    pub fn checksum(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for &b in self.raw {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }
}

/// Air the turbine is working in.
///
/// Explicit because density falls with elevation — this site is 484 m up, some
/// 5% thinner than sea level, which is 5% off the energy. A default would hide
/// that inside a number that looks measured.
#[derive(Clone, Copy, Debug)]
pub struct Air {
    /// Density, kg/m³.
    pub density_kg_m3: f64,
}

/// A wind turbine, as a power curve.
///
/// ⚠ **Deliberately has no `Default`.** Energy is a property of a wind year
/// *and* a machine, and a stage that invents the machine produces a number
/// nobody chose. Same rule as `cf-nebraska` refusing to hand back a value the
/// sources do not agree on.
#[derive(Clone, Copy, Debug)]
pub struct Turbine {
    /// What to call it.
    pub name: &'static str,
    /// Rotor diameter, metres.
    pub rotor_diameter_m: f64,
    /// Nameplate electrical power, watts.
    pub rated_power_w: f64,
    /// Below this wind speed the machine produces nothing, m/s.
    pub cut_in_ms: f64,
    /// At or above this the machine shuts down, m/s.
    pub cut_out_ms: f64,
    /// Overall coefficient of performance below rated — aerodynamic, drivetrain
    /// and generator together.
    ///
    /// ⚠ A single constant is the minimal model, not a faithful one: real Cp
    /// varies with tip-speed ratio and pitch. It is one number so the sweep can
    /// move it and show whether this stage matters, before anyone spends effort
    /// on a curve.
    pub cp: f64,
}

impl Turbine {
    /// Swept area, m².
    #[must_use]
    pub fn swept_area_m2(&self) -> f64 {
        core::f64::consts::PI * (self.rotor_diameter_m / 2.0).powi(2)
    }

    /// Electrical power at one wind speed, watts.
    #[must_use]
    pub fn power_w(&self, speed_ms: f64, air: Air) -> f64 {
        if speed_ms < self.cut_in_ms || speed_ms >= self.cut_out_ms {
            return 0.0;
        }
        let available = 0.5 * air.density_kg_m3 * self.swept_area_m2() * speed_ms.powi(3) * self.cp;
        available.min(self.rated_power_w)
    }
}

/// Anything that turns a wind speed into electrical power.
///
/// Exists so the chain can run on a **published power curve** or on the
/// parametric model and compare them, rather than being told which to trust.
/// A measurement of how far apart they are is worth more than a preference:
/// here the parametric model lands within 3.4% of the real curve over a year,
/// which is the sweep saying this term is not the weak one.
pub trait Machine {
    /// Electrical power at one wind speed, watts.
    fn power_w(&self, speed_ms: f64, air: Air) -> f64;
    /// Nameplate electrical power, watts.
    fn rated_power_w(&self) -> f64;
    /// Below this speed the machine produces nothing, m/s.
    fn cut_in_ms(&self) -> f64;
    /// At or above this the machine shuts down, m/s.
    fn cut_out_ms(&self) -> f64;
    /// What to call it.
    fn name(&self) -> &'static str;
}

impl Machine for Turbine {
    fn power_w(&self, speed_ms: f64, air: Air) -> f64 {
        Self::power_w(self, speed_ms, air)
    }
    fn rated_power_w(&self) -> f64 {
        self.rated_power_w
    }
    fn cut_in_ms(&self) -> f64 {
        self.cut_in_ms
    }
    fn cut_out_ms(&self) -> f64 {
        self.cut_out_ms
    }
    fn name(&self) -> &'static str {
        self.name
    }
}

/// A manufacturer's measured power curve, interpolated.
///
/// ⚠ **Not** a model — a table of what one machine actually produced on test.
/// Its Cp varies from 0.32 at cut-in to 0.47 at 8 m/s and down to 0.05 at
/// cut-out, which is the thing a single constant cannot represent and the
/// reason to carry the real one.
#[derive(Clone, Copy, Debug)]
pub struct PowerCurve {
    /// Machine name as the source names it.
    pub name: &'static str,
    /// Rotor diameter, metres.
    pub rotor_diameter_m: f64,
    /// Nameplate electrical power, watts.
    pub rated_power_w: f64,
    /// Air density the curve was measured or normalised at, kg/m³.
    ///
    /// ⚠⚠ **A recorded assumption, not a figure from the file.** The source CSV
    /// carries speed, power and Cp and states no density; 1.225 kg/m³ is the
    /// standard sea-level reference power curves are normally quoted at. If
    /// that is wrong for this machine, every energy figure derived from it is
    /// wrong by the density ratio — which is why it is a field with a warning
    /// rather than a constant buried in the arithmetic.
    pub reference_density_kg_m3: f64,
    /// Where the curve came from and under what terms.
    pub source: CurveSource,
    points: &'static [(f64, f64)],
}

/// Where a published power curve came from, and what its licence requires.
///
/// Grouped rather than three loose fields so provenance travels with the curve
/// and cannot be half-supplied — and so the constructor does not take eight
/// positional arguments, four of them strings, which is an invitation to
/// transpose two of them silently.
#[derive(Clone, Copy, Debug)]
pub struct CurveSource {
    /// Where the curve came from.
    pub origin: &'static str,
    /// Licence the source data is published under.
    pub licence: &'static str,
    /// ISO date the curve was retrieved.
    pub retrieved: &'static str,
}

impl PowerCurve {
    /// Build a curve from a published (speed m/s, power W) table.
    ///
    /// Public because a power curve is a **seam**: a farm choosing a different
    /// machine must be able to bring its own table without editing this crate.
    /// A curve that only exists as the one constant shipped here would make the
    /// machine a hard-coded choice, which is exactly what `Turbine` having no
    /// `Default` was meant to prevent.
    ///
    /// ⚠ `points` must be in ascending speed order; [`PowerCurve::power_w`]
    /// walks them in order and will interpolate nonsense otherwise.
    /// `the_published_curve_matches_its_source` checks the shipped one.
    #[must_use]
    pub const fn new(
        name: &'static str,
        rotor_diameter_m: f64,
        rated_power_w: f64,
        reference_density_kg_m3: f64,
        source: CurveSource,
        points: &'static [(f64, f64)],
    ) -> Self {
        Self {
            name,
            rotor_diameter_m,
            rated_power_w,
            reference_density_kg_m3,
            source,
            points,
        }
    }

    /// The (speed, power) points as published, in ascending speed order.
    #[must_use]
    pub const fn points(&self) -> &'static [(f64, f64)] {
        self.points
    }
}

impl Machine for PowerCurve {
    /// Linear interpolation between published points, zero outside them.
    ///
    /// ⚠ Air density is applied as a simple ratio, `P x (rho / rho_ref)`.
    /// IEC 61400-12-1 instead shifts the speed axis for pitch-regulated
    /// machines, and the two differ. The simple ratio is used because it is
    /// legible and because this stage's job is to be swappable, not to be the
    /// last word — but it IS an approximation and is written down as one.
    fn power_w(&self, speed_ms: f64, air: Air) -> f64 {
        let (first, last) = match (self.points.first(), self.points.last()) {
            (Some(f), Some(l)) => (*f, *l),
            _ => return 0.0,
        };
        if speed_ms < first.0 || speed_ms > last.0 {
            return 0.0;
        }
        let mut p = last.1;
        for w in self.points.windows(2) {
            let ((v0, p0), (v1, p1)) = (w[0], w[1]);
            if speed_ms <= v1 {
                let t = if (v1 - v0).abs() < f64::EPSILON {
                    0.0
                } else {
                    (speed_ms - v0) / (v1 - v0)
                };
                p = p0 + t * (p1 - p0);
                break;
            }
        }
        p * (air.density_kg_m3 / self.reference_density_kg_m3)
    }
    fn rated_power_w(&self) -> f64 {
        self.rated_power_w
    }
    fn cut_in_ms(&self) -> f64 {
        self.points.first().map_or(0.0, |p| p.0)
    }
    fn cut_out_ms(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.0)
    }
    fn name(&self) -> &'static str {
        self.name
    }
}

/// EWT DW54X — a 1 MW direct-drive machine built for distributed wind.
///
/// Chosen because MISSION's constraint is *fuel it yourself*: this is the class
/// of turbine a single farm actually installs, not a utility-scale machine from
/// a wind farm somebody else owns.
///
/// Data: NREL's `turbine-models` repository, BSD 3-Clause, Copyright 2020
/// Alliance for Sustainable Energy, LLC. See this repository's NOTICE.
pub const EWT_DW54X: PowerCurve = PowerCurve {
    name: "EWT DW54X, 1 MW, 54.1 m rotor",
    rotor_diameter_m: 54.1,
    rated_power_w: 1.0e6,
    reference_density_kg_m3: 1.225,
    source: CurveSource {
        origin: "https://github.com/NatLabRockies/turbine-models \
                 turbine_models/data/Distributed/EWT_DW54X_1MW_54.1.csv",
        licence: "BSD-3-Clause, Copyright 2020 Alliance for Sustainable Energy, LLC",
        retrieved: "2026-09-18",
    },
    points: &[
        (3.0, 12_000.0),
        (4.0, 39_000.0),
        (5.0, 78_000.0),
        (6.0, 138_000.0),
        (7.0, 222_000.0),
        (8.0, 337_000.0),
        (9.0, 464_000.0),
        (10.0, 597_000.0),
        (11.0, 743_000.0),
        (12.0, 881_000.0),
        (13.0, 960_000.0),
        (14.0, 997_000.0),
        (15.0, 1_000_000.0),
        (16.0, 1_000_000.0),
        (17.0, 1_000_000.0),
        (18.0, 1_000_000.0),
        (19.0, 1_000_000.0),
        (20.0, 1_000_000.0),
        (21.0, 1_000_000.0),
        (22.0, 1_000_000.0),
        (23.0, 1_000_000.0),
        (24.0, 1_000_000.0),
        (25.0, 1_000_000.0),
    ],
};

/// What a turbine took from a wind year.
#[derive(Clone, Copy, Debug)]
pub struct AnnualEnergy {
    /// Electrical energy over the year, kWh.
    pub kwh: f64,
    /// Fraction of nameplate output actually achieved.
    pub capacity_factor: f64,
    /// Samples below cut-in.
    pub samples_becalmed: usize,
    /// Samples at or above cut-out.
    pub samples_stormbound: usize,
    /// Samples clipped at rated power.
    pub samples_at_rated: usize,
}

/// Integrate a turbine's output over a wind year.
///
/// Every input is explicit and swappable — the year, the machine and the air —
/// because the plan for this chain seams **every** stage rather than only the
/// one believed weakest. A seam at the term you already suspect can only
/// confirm you.
#[must_use]
pub fn annual_energy<M: Machine + ?Sized>(year: &WindYear, machine: &M, air: Air) -> AnnualEnergy {
    let mut joules = 0.0;
    let (mut becalmed, mut stormbound, mut at_rated) = (0usize, 0usize, 0usize);
    let mut n = 0.0f64;
    for v in year.speeds() {
        n += 1.0;
        if v < machine.cut_in_ms() {
            becalmed += 1;
        } else if v >= machine.cut_out_ms() {
            stormbound += 1;
        }
        let p = machine.power_w(v, air);
        if p >= machine.rated_power_w() {
            at_rated += 1;
        }
        joules += p * year.interval_seconds();
    }
    let kwh = joules / 3.6e6;
    let hours = n * year.interval_seconds() / 3600.0;
    let capacity_factor = if hours > 0.0 && machine.rated_power_w() > 0.0 {
        kwh / (machine.rated_power_w() / 1000.0 * hours)
    } else {
        0.0
    };
    AnnualEnergy {
        kwh,
        capacity_factor,
        samples_becalmed: becalmed,
        samples_stormbound: stormbound,
        samples_at_rated: at_rated,
    }
}

/// What the nearest observing station says about the model, here.
///
/// A reanalysis is "validated against observation networks" in general. That
/// sentence is true and says nothing about *this* grid point, which is the only
/// place this chain's number comes from. These fields are the comparison
/// actually run, against the nearest station there is.
///
/// ⚠ **The tests pin these values; they cannot re-derive them offline.** The
/// observations are not checked in — they are a one-time validation of an
/// input, not an input themselves, and carrying another 1.8 MB to re-prove a
/// fixed result would be storage for its own sake. [`Corroboration::reproduce`]
/// is the executable referent.
#[derive(Clone, Copy, Debug)]
pub struct Corroboration {
    /// NOAA/NCEI station identifier.
    pub station_id: &'static str,
    /// Station name as NCEI records it.
    pub station_name: &'static str,
    /// Station latitude, degrees north.
    pub station_latitude: f64,
    /// Station longitude, degrees east.
    pub station_longitude: f64,
    /// Station elevation, metres — compare with the grid point's 484 m.
    pub station_elevation_m: f64,
    /// Distance from the modelled grid point, kilometres.
    pub km_from_grid_point: f64,
    /// Height the station's anemometer reports at, metres.
    pub observed_height_m: f64,
    /// Hours in 2012 with both a quality-passed observation and a modelled value.
    pub overlapping_hours: usize,
    /// Mean observed speed over those hours, m/s, at `observed_height_m`.
    pub observed_mean_ms: f64,
    /// Mean modelled speed over **the same hours**, m/s, at 100 m.
    ///
    /// ⚠ Not the full-year mean. Restricted to the overlapping hours, because
    /// comparing a subset against a whole-year figure would flatter or damn the
    /// model for the wrong reason.
    pub modelled_mean_ms: f64,
    /// Power-law exponent implied by the two means over the 10 m → 100 m span.
    ///
    /// Open farmland is usually quoted at 0.14–0.20. Slightly above that is what
    /// a site with stable nocturnal boundary layers would give, and the northern
    /// Plains have them. Corroboration, not proof.
    pub implied_shear_exponent: f64,
    /// Pearson correlation of hourly means, observed against modelled.
    ///
    /// ⚠ 0.61 is moderate, and it is the honest headline of this comparison: a
    /// 2 km reanalysis tracks a point observation's hour-to-hour variation only
    /// loosely. It corroborates the resource, not any individual hour — which
    /// matters downstream, because hydrogen storage is sized by *when* the wind
    /// blows and not only by how much of it there is.
    pub hourly_correlation: f64,
    /// ISO date the observations were pulled.
    pub retrieved: &'static str,
    /// How to obtain the observations again.
    pub reproduce: &'static str,
}

/// The model against the station 1.96 km away, over 2012.
pub const CARRINGTON_AIRPORT: Corroboration = Corroboration {
    station_id: "72073700266",
    station_name: "CARRINGTON MUNICIPAL AIRPORT, ND US",
    station_latitude: 47.451,
    station_longitude: -99.151,
    station_elevation_m: 490.1,
    km_from_grid_point: 1.96,
    observed_height_m: 10.0,
    overlapping_hours: 7128,
    observed_mean_ms: 4.570_010_288_065_844,
    modelled_mean_ms: 7.401_775_626_636_739,
    implied_shear_exponent: 0.209_418_738_233_836,
    hourly_correlation: 0.609_870_968_660_793,
    retrieved: "2026-09-18",
    reproduce: "GET https://www.ncei.noaa.gov/access/services/data/v1 with \
                dataset=global-hourly, stations=72073700266, \
                startDate=2012-01-01, endDate=2012-12-31, dataTypes=WND, \
                format=csv; parse the WND field's 4th component (speed in m/s \
                x10), keep quality codes 1 and 5, average to hourly, and compare \
                against this crate's samples averaged to the same hours",
};

/// 2012 at a grid point in Foster County, North Dakota.
///
/// Chosen because MISSION's farm is on the northern Plains and this is the
/// windiest of the candidate sites — if the fuel chain fails to close here, it
/// fails everywhere, which makes it the honest place to test it.
pub const FOSTER_COUNTY_ND_2012: WindYear = WindYear {
    year: 2012,
    site: Site {
        index: 1_151_805,
        latitude: 47.4567,
        longitude: -99.1264,
        state: "North Dakota",
        county: "Foster",
        elevation_m: 484,
        nearest_town: "Carrington, North Dakota",
        km_from_town: 0.78,
        measurement_height_m: 100.0,
    },
    provenance: Provenance {
        bucket: "nrel-pds-wtk",
        key: "conus/v1.0.0/2012/wtk_conus_2012_100m.h5",
        dataset: "windspeed_100m",
        site_index: 1_151_805,
        scale_factor: 100.0,
        fill_value: 65535,
        retrieved: "2026-09-18",
        sha256: "ee4a8d6b000e77f486f7fdfb85d6bd3584f60df6dec0ff393b37fd87b7f1194a",
        reproduce: "open https://nrel-pds-wtk.s3.amazonaws.com/conus/v1.0.0/2012/\
                    wtk_conus_2012_100m.h5 with h5py over a ranged-read file object, \
                    take windspeed_100m[:, 1151805], multiply by 100, round, and \
                    write little-endian u16",
    },
    raw: SAMPLES,
};
