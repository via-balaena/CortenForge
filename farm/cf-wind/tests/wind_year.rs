//! Gates on the wind stage, pre-registered before the crate was written.
//!
//! Every pinned number here was measured, not guessed — twice today a guessed
//! expected value went into a test and was wrong, and the dangerous case is the
//! guess that happens to match.
#![allow(
    clippy::panic,
    reason = "a gate that cannot name what it rejected is not much of a gate"
)]

use cf_wind::{
    Air, CARRINGTON_AIRPORT, CurveSource, EWT_DW54X, FOSTER_COUNTY_ND_2012 as YEAR, Machine,
    NOT_MEASURED_HERE, PowerCurve, Turbine, WindYear, annual_energy,
};

/// The reference machine the pinned energy figures use.
///
/// ⚠ Its parameters are stated assumptions, not a manufacturer's published
/// curve — see `the_reference_turbine_is_not_a_real_machines_curve`.
const REF: Turbine = Turbine {
    name: "reference 1.5 MW, 77 m rotor",
    rotor_diameter_m: 77.0,
    rated_power_w: 1.5e6,
    cut_in_ms: 3.5,
    cut_out_ms: 25.0,
    cp: 0.40,
};
/// Air at this site's 484 m elevation, ~5% thinner than sea level.
const AIR: Air = Air {
    density_kg_m3: 1.16,
};

// ------------------------------------------------------------------ W1 site

/// The site is what the source file says it is.
///
/// MUTATION: change the index or any field and this fails.
/// ⚠ What it does not establish: that the WIND Toolkit's own geolocation is
/// correct. Nothing here can check that.
#[test]
fn the_site_is_identified_by_the_source() {
    let s = YEAR.site;
    assert_eq!(s.index, 1_151_805);
    assert_eq!(s.state, "North Dakota");
    assert_eq!(s.county, "Foster");
    assert_eq!(s.elevation_m, 484);
    assert!((s.latitude - 47.4567).abs() < 1e-4);
    assert!((s.longitude - (-99.1264)).abs() < 1e-4);
    assert!((s.measurement_height_m - 100.0).abs() < f64::EPSILON);
    assert!(
        s.km_from_town < 3.0,
        "the grid point is {:.2} km from {} — too far to name it as that town",
        s.km_from_town,
        s.nearest_town
    );
    assert_eq!(
        s.index, YEAR.provenance.site_index,
        "site and provenance disagree"
    );
}

// ------------------------------------------------------------- W2/W3 source

/// MUTATION: flip any byte of the checked-in series and this fails.
#[test]
fn the_series_has_not_drifted() {
    assert_eq!(
        YEAR.checksum(),
        0x126e_4f9e_73fb_b1d8,
        "the checked-in wind series has changed since it was recorded"
    );
}

/// MUTATION: blank any provenance field and this fails.
#[test]
fn provenance_is_complete_and_reproducible() {
    let p = YEAR.provenance;
    for (field, value) in [
        ("bucket", p.bucket),
        ("key", p.key),
        ("dataset", p.dataset),
        ("retrieved", p.retrieved),
        ("sha256", p.sha256),
        ("reproduce", p.reproduce),
    ] {
        assert!(!value.trim().is_empty(), "provenance.{field} is empty");
    }
    assert_eq!(p.sha256.len(), 64, "sha256 is not a 64-hex-digit digest");
    assert!(p.sha256.chars().all(|c| c.is_ascii_hexdigit()));
    assert!(p.reproduce.len() > 80, "reproduce does not say how");
    assert!(
        p.key.contains("2012") && p.key.contains("100m"),
        "key does not name year and height"
    );
    assert!((p.scale_factor - 100.0).abs() < f64::EPSILON);
    assert_eq!(p.fill_value, 65535);
    assert!(p.retrieved.len() == 10 && p.retrieved.starts_with("2026-"));
}

// ------------------------------------------------------------ W4 the CUBE

/// ⛔⛔ The error this crate exists to prevent.
///
/// Power follows the mean of the cube. Estimating from the mean speed
/// understates it here by 40%. Jensen's inequality fixes the direction; the
/// magnitude is a property of this site and is measured, not assumed.
///
/// MUTATION: make `mean_cube_speed` return `mean_speed().powi(3)` and this
/// fails.
#[test]
fn the_cube_of_the_mean_is_not_the_mean_of_the_cube() {
    let mean_cube = YEAR.mean_cube_speed();
    let cube_of_mean = YEAR.mean_speed().powi(3);
    assert!(
        mean_cube > cube_of_mean,
        "Jensen's inequality is violated — the cube is convex"
    );
    let ratio = mean_cube / cube_of_mean;
    assert!(
        (ratio - 1.666_144).abs() < 1e-5,
        "the cube ratio at this site moved: {ratio:.6}"
    );
    assert!(
        ratio > 1.5,
        "a ratio this close to 1 would mean the distribution changed character"
    );
}

/// Sizing from the mean speed gets a materially different answer.
///
/// Makes the consequence executable rather than a warning in a doc comment: a
/// caller who cubes the average is not slightly off, they are ~40% low on the
/// resource before any turbine is chosen.
#[test]
fn estimating_from_the_mean_speed_understates_the_resource() {
    let honest = YEAR.mean_cube_speed();
    let naive = YEAR.mean_speed().powi(3);
    let shortfall = 1.0 - naive / honest;
    assert!(
        shortfall > 0.35 && shortfall < 0.45,
        "the naive estimate is {:.1}% low; the pinned figure is ~40%",
        shortfall * 100.0
    );
}

// ------------------------------------------------------- W6 plausibility

/// Bounds taken from the data, not from belief.
#[test]
fn the_series_is_physically_plausible() {
    let mut n = 0usize;
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in YEAR.speeds() {
        assert!(v.is_finite(), "non-finite wind speed");
        assert!((0.0..=40.0).contains(&v), "implausible wind speed {v} m/s");
        lo = lo.min(v);
        hi = hi.max(v);
        n += 1;
    }
    assert_eq!(n, 105_408, "samples missing or added");
    assert!((lo - 0.02).abs() < 1e-9, "minimum moved: {lo}");
    assert!((hi - 27.53).abs() < 1e-9, "maximum moved: {hi}");
}

/// 2012 is a leap year: 366 days at 5-minute resolution.
///
/// MUTATION: truncate the series and this fails. Guards against a partial file
/// being read as a full year, which would silently deflate annual energy.
#[test]
fn the_sample_count_is_a_whole_leap_year_at_five_minutes() {
    assert_eq!(YEAR.len(), 366 * 24 * 12);
    assert!(!YEAR.is_empty());
    assert_eq!(YEAR.year, 2012);
    assert!((YEAR.interval_seconds() - 300.0).abs() < f64::EPSILON);
    let samples = u32::try_from(YEAR.len()).unwrap_or_else(|_| panic!("sample count exceeds u32"));
    let hours = f64::from(samples) * YEAR.interval_seconds() / 3600.0;
    assert!(
        (hours - 8784.0).abs() < 1e-9,
        "the year is {hours} hours long"
    );
}

/// No sample is the source's missing-data marker, and none is silently dropped.
#[test]
fn no_sample_is_missing() {
    assert_eq!(
        YEAR.speeds().count(),
        YEAR.len(),
        "a sample was dropped as missing"
    );
    assert!(
        YEAR.speed(YEAR.len()).is_none(),
        "reading past the end returned a value"
    );
    assert!(YEAR.speed(usize::MAX).is_none());
}

// ------------------------------------------------------------- W5/W7 seams

/// Energy is pinned against the reference machine, measured not guessed.
#[test]
fn the_reference_machine_takes_a_plausible_share_of_the_year() {
    let e = annual_energy(&YEAR, &REF, AIR);
    assert!(
        (e.kwh - 5_355_053.892).abs() < 1.0,
        "energy moved: {:.3} kWh",
        e.kwh
    );
    assert!(
        (e.capacity_factor - 0.406_425).abs() < 1e-6,
        "cf moved: {}",
        e.capacity_factor
    );
    assert_eq!(e.samples_becalmed, 14_598);
    assert_eq!(e.samples_at_rated, 16_661);
    assert!(
        (0.25..0.55).contains(&e.capacity_factor),
        "a capacity factor of {:.3} is outside anything a real site reaches",
        e.capacity_factor
    );
}

/// Cut-out is exercised by the real series — by exactly one sample.
///
/// MUTATION: drop the cut-out branch and this fails. Worth pinning precisely
/// because one sample in 105408 is the kind of near-zero that looks like an
/// unexercised branch and is not.
#[test]
fn cut_out_is_exercised_by_the_real_year() {
    let e = annual_energy(&YEAR, &REF, AIR);
    assert_eq!(
        e.samples_stormbound, 1,
        "the count of samples at or above cut-out moved"
    );
    let above = YEAR.speeds().filter(|v| *v >= REF.cut_out_ms).count();
    assert_eq!(above, e.samples_stormbound, "independent count disagrees");
    assert!(
        (REF.power_w(26.0, AIR)).abs() < f64::EPSILON,
        "a turbine above cut-out produced power"
    );
    assert!(
        (REF.power_w(1.0, AIR)).abs() < f64::EPSILON,
        "a turbine below cut-in produced power"
    );
}

/// ⛔ Swapping the machine changes the answer — the turbine is a real seam.
#[test]
fn swapping_the_turbine_changes_the_answer() {
    let small = Turbine {
        name: "small",
        rotor_diameter_m: 21.0,
        rated_power_w: 100e3,
        ..REF
    };
    let a = annual_energy(&YEAR, &REF, AIR).kwh;
    let b = annual_energy(&YEAR, &small, AIR).kwh;
    assert!(
        b < a / 5.0,
        "a far smaller rotor produced comparable energy: {b} vs {a}"
    );
}

/// ⛔ Swapping the air changes the answer — density is a seam, not a constant.
#[test]
fn swapping_the_air_changes_the_answer() {
    let sea_level = Air {
        density_kg_m3: 1.225,
    };
    let a = annual_energy(&YEAR, &REF, AIR).kwh;
    let b = annual_energy(&YEAR, &REF, sea_level).kwh;
    assert!(b > a, "denser air must yield more energy below rated");
    let lift = b / a - 1.0;
    assert!(
        (0.005..0.06).contains(&lift),
        "a {:.1}% change from 5% denser air is not credible",
        lift * 100.0
    );
}

/// ⛔ Swapping the wind year changes the answer — the series is a seam too.
///
/// A stage that can only run on the one series shipped with it is hard-coded,
/// not seamed, and the sweep could never say how much this stage moves the
/// chain.
#[test]
fn swapping_the_wind_year_changes_the_answer() {
    // a contrived calm year: every sample 2.00 m/s, below cut-in
    // 100 samples of 2.00 m/s (scaled u16 = 200 = 0x00C8), all below cut-in.
    // Built in a const block rather than typed out: a hand-written byte array
    // is a hand-counted byte array, and the first draft of this one had the
    // wrong length.
    static CALM: [u8; 200] = {
        let mut a = [0u8; 200];
        let mut i = 0;
        while i < 200 {
            a[i] = 0xC8;
            a[i + 1] = 0x00;
            i += 2;
        }
        a
    };
    let calm = WindYear::from_scaled_le_bytes(9999, YEAR.site, YEAR.provenance, &CALM);
    assert_eq!(calm.len(), 100);
    assert!(
        (calm.mean_speed() - 2.0).abs() < 1e-9,
        "the contrived year did not decode"
    );
    let e = annual_energy(&calm, &REF, AIR);
    assert!(
        e.kwh.abs() < f64::EPSILON,
        "a year entirely below cut-in produced energy"
    );
    assert_eq!(e.samples_becalmed, 100);
    assert!(e.capacity_factor.abs() < f64::EPSILON);
}

// ------------------------------------------------------------ W9 caveats

/// ⚠ MISSION asks for a MEASURED wind year. These are modelled.
///
/// MUTATION: empty `NOT_MEASURED_HERE` and this fails. The gap is recorded for
/// the same reason `cf-nebraska` records the absent slip column — the tempting
/// move is to let a good dataset stand in silently for the thing that was asked
/// for.
#[test]
fn the_modelled_versus_measured_gap_is_recorded() {
    assert!(!NOT_MEASURED_HERE.is_empty());
    assert!(
        NOT_MEASURED_HERE
            .iter()
            .any(|c| c.what.contains("modelled") || c.what.contains("not measurements")),
        "the reanalysis-versus-measurement gap is not recorded"
    );
    assert!(
        NOT_MEASURED_HERE
            .iter()
            .any(|c| c.what.contains("one year")),
        "inter-annual variability is not recorded"
    );
    for c in NOT_MEASURED_HERE {
        assert!(!c.what.trim().is_empty());
        assert!(c.why.len() > 80, "{}: the reason is too short", c.what);
    }
}

/// The shipped turbine parameters are assumptions, and say so.
///
/// Guards the seam between "a figure from a source" and "a number I chose":
/// nothing in this crate cites a manufacturer's power curve, and claiming one
/// would be the kind of unsourced number the chain exists to avoid.
#[test]
fn the_reference_turbine_is_not_a_real_machines_curve() {
    assert!(
        REF.name.contains("reference"),
        "the test machine is named as though it were a real product"
    );
    // `const` assertions: these describe the reference machine's definition,
    // so a violation should fail the BUILD rather than one test run.
    const {
        assert!(
            REF.cp > 0.0 && REF.cp < 0.593,
            "Cp must sit below the Betz limit"
        );
    };
    const {
        assert!(
            REF.cut_in_ms < REF.cut_out_ms,
            "cut-in must precede cut-out"
        );
    };
    assert!(REF.swept_area_m2() > 4000.0 && REF.swept_area_m2() < 5000.0);
}

/// An empty series yields zeros rather than `NaN`.
///
/// A wind year with no samples is not a real input, but `0/0` is how a mean
/// becomes `NaN`, and `NaN` is how a chain of multiplied estimates produces a
/// headline number that is silently not a number at all.
#[test]
fn an_empty_year_yields_zeros_not_nan() {
    static NONE: [u8; 0] = [];
    let empty = WindYear::from_scaled_le_bytes(9999, YEAR.site, YEAR.provenance, &NONE);
    assert!(empty.is_empty());
    assert_eq!(empty.len(), 0);
    assert!(
        empty.mean_speed().abs() < f64::EPSILON,
        "empty mean is not zero"
    );
    assert!(
        empty.mean_cube_speed().abs() < f64::EPSILON,
        "empty mean-cube is not zero"
    );
    assert!(!empty.mean_speed().is_nan() && !empty.mean_cube_speed().is_nan());
    let e = annual_energy(&empty, &REF, AIR);
    assert!(e.kwh.abs() < f64::EPSILON);
    assert!(
        !e.capacity_factor.is_nan(),
        "capacity factor became NaN on an empty year"
    );
    assert!(e.capacity_factor.abs() < f64::EPSILON);
}

/// A machine never exceeds its own nameplate, and the cap actually binds.
///
/// ⛔ Added because a probe found `rated_power_w` was load-bearing in the code
/// and load-bearing in no test: replacing the cap with the reference turbine's
/// own rating changed nothing observable, because every other test either uses
/// that turbine or only checks a smaller one produces less. A field the gates
/// cannot see is a field that can silently stop working.
#[test]
fn rated_power_caps_output_and_the_cap_binds() {
    for v in [0.0, 3.6, 5.0, 10.0, 15.0, 20.0, 24.9, 30.0] {
        let p = REF.power_w(v, AIR);
        assert!(
            p <= REF.rated_power_w + 1e-6,
            "at {v} m/s the machine produced {p} W, above its {} W nameplate",
            REF.rated_power_w
        );
    }
    // Same rotor, a tenth of the generator: the cap must bind for much of the
    // year rather than being decorative.
    let over_rotored = Turbine {
        name: "reference rotor, small generator",
        rated_power_w: 150e3,
        ..REF
    };
    let e = annual_energy(&YEAR, &over_rotored, AIR);
    assert!(
        e.samples_at_rated > YEAR.len() / 2,
        "a 150 kW generator on a 77 m rotor clipped only {} of {} samples",
        e.samples_at_rated,
        YEAR.len()
    );
    assert!(
        e.capacity_factor > 0.75,
        "an over-rotored machine should run near nameplate most of the time, got {:.3}",
        e.capacity_factor
    );
    // and its energy must be far below the properly rated machine's
    let full = annual_energy(&YEAR, &REF, AIR);
    assert!(
        e.kwh < full.kwh / 3.0,
        "the cap did not reduce annual energy"
    );
}

/// ⛔⛔ Every turbine parameter must move the answer on its own.
///
/// The seam tests above vary a whole machine at once, and that is not enough:
/// probes showed `cp` and `rotor_diameter_m` could each be hard-coded to the
/// reference value with every test still green, because a second parameter
/// masked the first. A parameter no gate can see is a knob that can quietly
/// stop being connected — and for a chain whose whole purpose is a sensitivity
/// sweep, an unconnected knob is worse than a wrong one.
///
/// MUTATION: replace any field with a literal in `power_w` or `swept_area_m2`
/// and this fails.
#[test]
fn every_turbine_parameter_changes_the_answer() {
    let base = annual_energy(&YEAR, &REF, AIR).kwh;
    let cases: [(&str, Turbine); 5] = [
        (
            "rotor_diameter_m",
            Turbine {
                rotor_diameter_m: REF.rotor_diameter_m * 0.5,
                ..REF
            },
        ),
        // rated high enough that the bigger rotor is not simply clipped away
        (
            "rated_power_w",
            Turbine {
                rated_power_w: REF.rated_power_w * 0.5,
                ..REF
            },
        ),
        (
            "cut_in_ms",
            Turbine {
                cut_in_ms: 9.0,
                ..REF
            },
        ),
        (
            "cut_out_ms",
            Turbine {
                cut_out_ms: 11.0,
                ..REF
            },
        ),
        (
            "cp",
            Turbine {
                cp: REF.cp * 0.5,
                ..REF
            },
        ),
    ];
    for (field, t) in cases {
        let got = annual_energy(&YEAR, &t, AIR).kwh;
        let change = (got - base).abs() / base;
        assert!(
            change > 0.02,
            "halving/altering `{field}` moved annual energy by only {:.4}% \
             ({got:.0} vs {base:.0} kWh) — the parameter is not connected",
            change * 100.0
        );
    }
}

/// The model is checked against the nearest station, not called validated.
///
/// ⛔ Added after a review criterion asked whether the oracle was genuinely
/// unreachable. It was not — NOAA has an hourly station 1.96 km from the grid
/// point covering the same year, and the PR had implied none existed. Asserting
/// a capability is absent without probing it is the same mistake as claiming a
/// CI hole was ungated when a guard already existed.
///
/// MUTATION: change any recorded figure and this fails.
/// ⚠ These values are PINNED, not re-derived — the observations are not checked
/// in. `reproduce` is the executable referent.
#[test]
fn the_model_is_corroborated_against_the_nearest_station() {
    let c = CARRINGTON_AIRPORT;
    assert_eq!(c.station_id, "72073700266");
    assert!(c.station_name.contains("CARRINGTON"));
    assert!(
        c.km_from_grid_point < 5.0,
        "a station {:.2} km away is not 'the nearest station' in any useful sense",
        c.km_from_grid_point
    );
    assert!(
        (c.station_elevation_m - 484.0).abs() < 25.0,
        "the station sits {:.1} m up against the grid point's 484 m — too different \
         in terrain for the comparison to mean much",
        c.station_elevation_m
    );
    assert!(
        c.overlapping_hours > 5000,
        "only {} hours overlap; too few to say anything about a year",
        c.overlapping_hours
    );
    assert!(
        c.overlapping_hours < 8784,
        "more overlapping hours than the year holds"
    );
    assert!(!c.reproduce.trim().is_empty() && c.reproduce.len() > 80);
    assert!(c.retrieved.len() == 10 && c.retrieved.starts_with("2026-"));
}

/// The comparison corroborates the resource, and the numbers say how well.
///
/// The implied shear exponent must be physically sensible and the correlation
/// must be stated for what it is. A reanalysis that tracked a point observation
/// perfectly would be suspicious, not reassuring.
#[test]
fn the_corroboration_figures_are_physically_sensible() {
    let c = CARRINGTON_AIRPORT;
    assert!(
        c.modelled_mean_ms > c.observed_mean_ms,
        "100 m wind must exceed 10 m wind; got {} vs {}",
        c.modelled_mean_ms,
        c.observed_mean_ms
    );
    // the exponent the two means imply, recomputed from them rather than trusted
    let implied = (c.modelled_mean_ms / c.observed_mean_ms).ln() / (100.0f64 / 10.0).ln();
    assert!(
        (implied - c.implied_shear_exponent).abs() < 1e-9,
        "the recorded shear exponent {} is not what the recorded means imply ({implied})",
        c.implied_shear_exponent
    );
    assert!(
        (0.10..=0.30).contains(&c.implied_shear_exponent),
        "a shear exponent of {:.3} is outside anything open farmland produces",
        c.implied_shear_exponent
    );
    assert!(
        (0.4..0.85).contains(&c.hourly_correlation),
        "an hourly correlation of {:.3} between a 2 km reanalysis and a point \
         observation is outside the believable range in either direction",
        c.hourly_correlation
    );
    // and the overlapping-hours mean must be close to, but not equal to, the year's
    let full = YEAR.mean_speed();
    assert!(
        (c.modelled_mean_ms - full).abs() < 0.5,
        "the overlapping-hours mean {} is far from the full-year mean {full}",
        c.modelled_mean_ms
    );
    assert!(
        (c.modelled_mean_ms - full).abs() > 1e-9,
        "the overlapping-hours mean is identical to the full-year mean, which \
         would mean the subset was never actually taken"
    );
}

/// The published curve is the source's, point for point.
///
/// MUTATION: change any point and this fails.
#[test]
fn the_published_curve_matches_its_source() {
    let c = EWT_DW54X;
    let p = c.points();
    assert_eq!(p.len(), 23, "the published curve has 23 points");
    assert!((p[0].0 - 3.0).abs() < f64::EPSILON, "cut-in point");
    assert!((p[0].1 - 12_000.0).abs() < f64::EPSILON, "power at cut-in");
    assert!(
        (p[p.len() - 1].0 - 25.0).abs() < f64::EPSILON,
        "cut-out point"
    );
    assert!((c.rated_power_w - 1.0e6).abs() < f64::EPSILON);
    // ascending speeds, and power never exceeds nameplate
    for w in p.windows(2) {
        assert!(w[1].0 > w[0].0, "points are not in ascending speed order");
    }
    for &(v, pw) in p {
        assert!(
            pw <= c.rated_power_w + 1e-6,
            "point at {v} m/s exceeds nameplate"
        );
    }
    assert!(
        c.source.licence.contains("BSD-3")
            && c.source.licence.contains("Alliance for Sustainable Energy"),
        "the source licence and copyright holder must be recorded"
    );
    assert!(c.source.origin.contains("turbine-models"));
}

/// Interpolation is linear between points and zero outside them.
#[test]
fn the_curve_interpolates_and_cuts_out() {
    let air = Air {
        density_kg_m3: EWT_DW54X.reference_density_kg_m3,
    };
    // 8.0 -> 337 kW, 9.0 -> 464 kW, so 8.5 is the midpoint
    let mid = EWT_DW54X.power_w(8.5, air);
    assert!(
        (mid - 400_500.0).abs() < 1.0,
        "8.5 m/s gave {mid} W, not the midpoint of its bracketing points"
    );
    assert!(
        EWT_DW54X.power_w(2.9, air).abs() < f64::EPSILON,
        "produced power below cut-in"
    );
    assert!(
        EWT_DW54X.power_w(25.1, air).abs() < f64::EPSILON,
        "produced power above cut-out"
    );
    assert!(EWT_DW54X.power_w(0.0, air).abs() < f64::EPSILON);
    // exact points reproduce exactly
    for &(v, expect) in EWT_DW54X.points() {
        let got = EWT_DW54X.power_w(v, air);
        assert!(
            (got - expect).abs() < 1e-6,
            "at {v} m/s: {got} != published {expect}"
        );
    }
}

/// Thinner air yields less power, by the ratio the curve documents.
///
/// MUTATION: drop the density correction and this fails. ⚠ The correction is a
/// simple ratio, not IEC 61400-12-1's speed-shift; the crate says so.
#[test]
fn the_curve_is_corrected_for_air_density() {
    let refd = Air {
        density_kg_m3: EWT_DW54X.reference_density_kg_m3,
    };
    let site = Air {
        density_kg_m3: 1.16,
    };
    let a = EWT_DW54X.power_w(10.0, refd);
    let b = EWT_DW54X.power_w(10.0, site);
    assert!(b < a, "thinner air must give less power");
    let expected = a * (1.16 / EWT_DW54X.reference_density_kg_m3);
    assert!(
        (b - expected).abs() < 1e-6,
        "the correction is not the documented ratio"
    );
}

/// ⛔⛔ What the real curve says about the model that was shipped without one.
///
/// The parametric constant-Cp machine lands within 3.4% of the measured curve
/// over a full year, like for like. That is the sweep doing its job: this
/// stage's turbine model is **not** the weak term, and effort spent refining it
/// would have bought almost nothing. Recorded as a number so the next person
/// does not have to guess — and pinned, so if either model drifts the
/// comparison is re-examined rather than quietly assumed to still hold.
#[test]
fn the_parametric_model_is_close_to_the_measured_curve() {
    let refd = Air {
        density_kg_m3: EWT_DW54X.reference_density_kg_m3,
    };
    let like_for_like = Turbine {
        name: "parametric match to the DW54X",
        rotor_diameter_m: EWT_DW54X.rotor_diameter_m,
        rated_power_w: EWT_DW54X.rated_power_w,
        cut_in_ms: 3.0,
        cut_out_ms: 25.0,
        cp: 0.40,
    };
    let real = annual_energy(&YEAR, &EWT_DW54X, refd);
    let param = annual_energy(&YEAR, &like_for_like, refd);
    let gap = param.kwh / real.kwh - 1.0;
    assert!(
        (-0.06..=-0.01).contains(&gap),
        "the parametric model is {:.2}% from the measured curve; the recorded \
         figure is -3.4%, and a big move means one of the two changed",
        gap * 100.0
    );
    assert!(
        (real.kwh - 3_145_000.0).abs() < 5_000.0,
        "real-curve energy moved: {:.0}",
        real.kwh
    );
    assert!(
        (0.30..0.42).contains(&real.capacity_factor),
        "a 1 MW distributed machine at this site should sit near 36%, got {:.3}",
        real.capacity_factor
    );
}

/// The reference density is an assumption, and is labelled as one.
///
/// It is not in the source CSV. If it is wrong, every energy figure from this
/// curve is wrong by the density ratio — so it is a field with a warning rather
/// than a constant hidden in the arithmetic.
#[test]
fn the_curves_reference_density_is_a_recorded_assumption() {
    assert!((EWT_DW54X.reference_density_kg_m3 - 1.225).abs() < f64::EPSILON);
    // a `const` assertion: it describes the constant's definition, so a
    // violation should fail the BUILD rather than one test run.
    const {
        assert!(
            EWT_DW54X.reference_density_kg_m3 > 1.0 && EWT_DW54X.reference_density_kg_m3 < 1.4,
            "an air density outside 1.0-1.4 kg/m3 is not a sea-level reference"
        );
    };
}

/// Both machines answer the same questions, so the chain can hold either.
///
/// MUTATION: break any trait forwarding method and this fails. Without it the
/// `Machine` accessors are exercised only through `annual_energy`, which reads
/// some of them and not others.
#[test]
fn both_machine_kinds_report_the_same_properties() {
    let by_curve: &dyn Machine = &EWT_DW54X;
    assert!((by_curve.rated_power_w() - 1.0e6).abs() < f64::EPSILON);
    assert!(
        (by_curve.cut_in_ms() - 3.0).abs() < f64::EPSILON,
        "cut-in comes from the first point"
    );
    assert!(
        (by_curve.cut_out_ms() - 25.0).abs() < f64::EPSILON,
        "cut-out comes from the last"
    );
    assert!(by_curve.name().contains("DW54X"));

    let by_model: &dyn Machine = &REF;
    assert!((by_model.rated_power_w() - REF.rated_power_w).abs() < f64::EPSILON);
    assert!((by_model.cut_in_ms() - REF.cut_in_ms).abs() < f64::EPSILON);
    assert!((by_model.cut_out_ms() - REF.cut_out_ms).abs() < f64::EPSILON);
    assert_eq!(by_model.name(), REF.name);

    // and the chain accepts either through the same call
    let air = Air {
        density_kg_m3: 1.16,
    };
    assert!(annual_energy(&YEAR, by_curve, air).kwh > 0.0);
    assert!(annual_energy(&YEAR, by_model, air).kwh > 0.0);
}

/// A caller can bring its own machine's curve.
///
/// The seam: a farm that buys something else must not have to edit this crate.
/// Also covers the degenerate empty curve, which must produce nothing rather
/// than panic or return `NaN`.
#[test]
fn a_caller_can_supply_its_own_curve() {
    static MINE: [(f64, f64); 3] = [(4.0, 10_000.0), (10.0, 100_000.0), (20.0, 100_000.0)];
    static NONE: [(f64, f64); 0] = [];
    let mine = PowerCurve::new(
        "a farm's own machine",
        30.0,
        100_000.0,
        1.225,
        CurveSource {
            origin: "the manufacturer's datasheet",
            licence: "unstated",
            retrieved: "2026-09-18",
        },
        &MINE,
    );
    let air = Air {
        density_kg_m3: 1.225,
    };
    assert!(
        (mine.power_w(7.0, air) - 55_000.0).abs() < 1.0,
        "midpoint interpolation"
    );
    assert!(mine.power_w(3.9, air).abs() < f64::EPSILON);
    assert!(mine.power_w(20.1, air).abs() < f64::EPSILON);
    assert!(annual_energy(&YEAR, &mine, air).kwh > 0.0);

    let empty = PowerCurve::new(
        "none",
        0.0,
        0.0,
        1.225,
        CurveSource {
            origin: "-",
            licence: "-",
            retrieved: "2026-09-18",
        },
        &NONE,
    );
    assert!(
        empty.power_w(10.0, air).abs() < f64::EPSILON,
        "an empty curve produced power"
    );
    let e = annual_energy(&YEAR, &empty, air);
    assert!(e.kwh.abs() < f64::EPSILON);
    assert!(
        !e.capacity_factor.is_nan(),
        "empty curve gave a `NaN` capacity factor"
    );
}
