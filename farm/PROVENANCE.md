# Where the farm chain's numbers come from — and on what terms

Every figure in `farm/` is read off a published source. This page lists all of them
in one place, with **what each publisher requires**, because that answer differs
source by source and the difference is not obvious.

⛔⛔ **"It is a government document, therefore it is free" is not a determination.**
Eight of the ten sources below are U.S. federal works with no copyright. The ninth is
a national-laboratory dataset under BSD-3, which puts a notice in `NOTICE`. The tenth
is a U.S. federal database that **asserts copyright** and is therefore never committed.
That spread is why each crate records the terms in a field rather than in a comment.

## The chain

| stage | crate | produces |
|---|---|---|
| calibration | `cf-nebraska` | measured drawbar, pull and fuel for one real tractor |
| 1. wind | `cf-wind` | kWh/yr at one real site |
| 2. electrolysis | `cf-electrolysis` | kg H₂/yr at the electrolyser outlet, ~21 bar |
| 3. compression + storage | `cf-storage` | kg H₂/yr at tank pressure, and the tank |

## Every source

| # | source | used by | terms | committed? |
|---|---|---|---|---|
| 1 | Nebraska OECD Tractor Test reports, editions 2016 / 2017 / 2019 ([2016](https://govdocs.nebraska.gov/epubs/U2060/S001-2016.pdf)) | `cf-nebraska` | State of Nebraska public documents | figures only |
| 2 | NREL **WIND Toolkit**, CONUS v1.0.0, `nrel-pds-wtk` S3 | `cf-wind` | U.S. Gov work, public domain | ✅ one grid point, one year (206 KiB) |
| 3 | NOAA/NCEI Integrated Surface Database, station 72073700266 — **wind** | `cf-wind` | U.S. Gov work, public domain | summary statistics |
| 4 | NOAA/NCEI Integrated Surface Database, station 72073700266 — **temperature** | `cf-storage` | U.S. Gov work, public domain | summary statistics |
| 5 | NREL **turbine-models**, `EWT_DW54X` power curve | `cf-wind` | **BSD-3-Clause**, © 2020 Alliance for Sustainable Energy | ✅ 23 points — ⚠ see `NOTICE` |
| 6 | DOE **Hydrogen Program Record 19009**, PEM electrolysis cost (2019) | `cf-electrolysis` | U.S. Gov work, 17 U.S.C. §105 | figures only |
| 7 | DOE **Alternative Fuels Data Center**, fuel properties | `cf-electrolysis` | U.S. Gov work, 17 U.S.C. §105 | figures only |
| 8 | DOE **Hydrogen Program Record 9013**, compression and liquefaction energy (2009) | `cf-storage` | U.S. Gov work, 17 U.S.C. §105 | figures only |
| 9 | **Goodwin, Diller, Roder & Weber**, J. Res. NBS **68A**(1) 121–126, 1964, [doi:10.6028/jres.068A.011](https://doi.org/10.6028/jres.068A.011) | `cf-storage` | U.S. Gov work, public domain (the paper says so) | Table 2 and two equations |
| 10 | **NIST Chemistry WebBook**, SRD 69, hydrogen fluid properties | `cf-storage` | ⛔ **Standard Reference Data Act — copyright ASSERTED** | ❌ **never** — see [`cf-storage/NIST_VALIDATION.md`](cf-storage/NIST_VALIDATION.md) |

## The two that need more than a row

### ⚠ Source 5 — the only one that obliges us

NREL's `turbine-models` is BSD-3, so redistributing the power curve requires
reproducing the copyright notice, the conditions and the disclaimer. That is why this
repository has a **THIRD-PARTY DATA** section in `NOTICE`. It is the only entry in it.

### ⛔ Source 10 — the loud exception

NIST Standard Reference Data is copyright-asserted **notwithstanding** 17 U.S.C. §105,
under the Standard Reference Data Act (15 U.S.C. §290e). It is used as a **retrieved
oracle**: fetched to a temp directory, compared against, deleted. What `cf-storage`
commits is the *measurement* of its own model's error, plus the command that
reproduces it — the pattern `design/cf-fsu-geometry/BODYPARTS3D.md` established.

## ★ Two sources that look independent and are not

Record 9013 (source 8) states that its theoretical compression figures were
*"determined from exergy differences using the standard properties … as reported by
NIST"* (source 10). **Agreeing with both is one check, not two.** `cf-storage`'s
independent leg is source 9, a 1964 NBS paper that supplies the model's only parameter
and touches neither.

⚠ This is the same trap `cf-electrolysis` found on a different axis: three
transcription checks that were all computed *inside the same column* are one check.
Before adding a redundancy, ask what frame of reference it shares with the others.

## How to verify a source is still what this repo says

Each crate carries the retrieval command with the figures — `Provenance::reproduce`
in `cf-wind`, `Source` in `cf-electrolysis` and `cf-storage`, `Edition` in
`cf-nebraska`. Committed binary data additionally carries a SHA-256 that is checked
with `shasum`; that is what ties `cf-wind`'s series to the bucket it came from, and
it is checked **after** a squash-merge, not only before.
