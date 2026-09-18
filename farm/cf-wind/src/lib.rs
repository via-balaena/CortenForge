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
//! physical model reconstructing the atmosphere, validated against observations
//! but not an anemometer on this farm. See [`NOT_MEASURED_HERE`].
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
              on a 2 km grid. It is validated against observation networks and \
              is the standard resource dataset, but no anemometer stood at this \
              grid point. MISSION asks for a farm's MEASURED wind year, so this \
              is a stand-in. Closing it means on-site measurement, or at minimum \
              comparison against a nearby observing station — which would also \
              quantify the model's bias here rather than assuming it is small.",
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
pub fn annual_energy(year: &WindYear, turbine: &Turbine, air: Air) -> AnnualEnergy {
    let mut joules = 0.0;
    let (mut becalmed, mut stormbound, mut at_rated) = (0usize, 0usize, 0usize);
    let mut n = 0.0f64;
    for v in year.speeds() {
        n += 1.0;
        if v < turbine.cut_in_ms {
            becalmed += 1;
        } else if v >= turbine.cut_out_ms {
            stormbound += 1;
        }
        let p = turbine.power_w(v, air);
        if p >= turbine.rated_power_w {
            at_rated += 1;
        }
        joules += p * year.interval_seconds();
    }
    let kwh = joules / 3.6e6;
    let hours = n * year.interval_seconds() / 3600.0;
    let capacity_factor = if hours > 0.0 && turbine.rated_power_w > 0.0 {
        kwh / (turbine.rated_power_w / 1000.0 * hours)
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
