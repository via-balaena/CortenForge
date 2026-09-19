# Where the farm chain's numbers come from — and on what terms

Every figure in `farm/` is read off a published source. This page lists all of them
in one place, with **what each publisher requires**, because that answer differs
source by source and the difference is not obvious.

⛔⛔ **"It is a government document, therefore it is free" is not a determination.**
Sixteen sources fall under **six** different sets of terms:

- **eleven** are U.S. *federal* works with no copyright (17 U.S.C. §105);
- **one** is a U.S. federal database that **asserts copyright** anyway, under the
  Standard Reference Data Act, and is therefore never committed;
- **one** is a national-laboratory dataset under **BSD-3**, which puts a notice in `NOTICE`;
- **one** is a **state** publication whose copyright status this repository has
  **not determined** — see below;
- **one** is an **international** body's table, where the determination is that
  the *values* are facts and the *tabulation* is theirs — see source 14;
- **one** is a **federal tool over a state source** — a USDA-ARS worksheet whose
  data the tool itself says comes from a state university's crop budgets. The
  federal wrapper does not launder the provenance — see source 16.

⚠ An earlier draft of this page counted eight federal works and folded the state
source in with them. §105 does not reach state works, so that was the very
inference the paragraph above warns against, made in its own opening line.

## The chain

| stage | crate | produces |
|---|---|---|
| calibration | `cf-nebraska` | measured drawbar, pull and fuel for one real tractor |
| 1. wind | `cf-wind` | kWh/yr at one real site |
| 2. electrolysis | `cf-electrolysis` | kg H₂/yr at the electrolyser outlet, ~21 bar |
| 3. compression + storage | `cf-storage` | kg H₂/yr at tank pressure, and the tank |
| 4. tillage window + demand | `cf-tillage` | the operating window, and the engine efficiency the season needs |
| 5. nitrogen demand | `cf-nitrogen` | the hydrogen an acre's fertilizer needs, and the acres one turbine covers |
| 6. acres per season | `cf-acres` | the three ceilings, and which one binds |

## Every source

| # | source | used by | terms | committed? |
|---|---|---|---|---|
| 1 | Nebraska OECD Tractor Test reports, editions 2016 / 2017 / 2019 ([2016](https://govdocs.nebraska.gov/epubs/U2060/S001-2016.pdf)) | `cf-nebraska` | ⚠ **NOT DETERMINED** — a *state* publication, so §105 does not apply | figures only |
| 2 | NREL **WIND Toolkit**, CONUS v1.0.0, `nrel-pds-wtk` S3 | `cf-wind` | U.S. Gov work, public domain (17 U.S.C. §105) | ✅ one grid point, one year (206 KiB) |
| 3 | NOAA/NCEI Integrated Surface Database, station 72073700266 — **wind** | `cf-wind` | U.S. Gov work, public domain (17 U.S.C. §105) | summary statistics |
| 4 | NOAA/NCEI Integrated Surface Database, station 72073700266 — **temperature** | `cf-storage` | U.S. Gov work, public domain (17 U.S.C. §105) | summary statistics |
| 5 | NREL **turbine-models**, `EWT_DW54X` power curve | `cf-wind` | **BSD-3-Clause**, © 2020 Alliance for Sustainable Energy | ✅ 23 points — ⚠ see `NOTICE` |
| 6 | DOE **Hydrogen Program Record 19009**, PEM electrolysis cost (2019) | `cf-electrolysis` | U.S. Gov work, public domain (17 U.S.C. §105) | figures only |
| 7 | DOE **Alternative Fuels Data Center**, fuel properties | `cf-electrolysis`, `cf-tillage` | U.S. Gov work, public domain (17 U.S.C. §105) | figures only |
| 8 | DOE **Hydrogen Program Record 9013**, compression and liquefaction energy (2009) | `cf-storage` | U.S. Gov work, public domain (17 U.S.C. §105) | figures only |
| 9 | **Goodwin, Diller, Roder & Weber**, J. Res. NBS **68A**(1) 121–126, 1964, [doi:10.6028/jres.068A.011](https://doi.org/10.6028/jres.068A.011) | `cf-storage` | U.S. Gov work, public domain (17 U.S.C. §105) | Table 2 and two equations |
| 10 | **NIST Chemistry WebBook**, SRD 69, hydrogen fluid properties | `cf-storage` | ⛔ **Standard Reference Data Act — copyright ASSERTED** | ❌ **never** — see [`cf-storage/NIST_VALIDATION.md`](cf-storage/NIST_VALIDATION.md) |
| 11 | **USDA NASS Quick Stats**, crops — North Dakota weekly fieldwork and harvest progress | `cf-tillage` | U.S. Gov work, public domain (17 U.S.C. §105) | ✅ 921 rows (21 KiB) — see [`cf-tillage/NASS_VALIDATION.md`](cf-tillage/NASS_VALIDATION.md) |
| 12 | **EPA NR-005c**, EPA420-P-04-005, nonroad load factors and annual activity (2004) | `cf-tillage` | U.S. Gov work, public domain (17 U.S.C. §105) | figures only |

| 13 | **USDA NASS Quick Stats**, environmental — North Dakota fertilizer application (Agricultural Chemical Use Program) | `cf-nitrogen` | U.S. Gov work, public domain (17 U.S.C. §105) | ✅ 356 rows (22 KiB) — see [`cf-nitrogen/NASS_FERTILIZER.md`](cf-nitrogen/NASS_FERTILIZER.md) |
| 14 | **IUPAC / CIAAW**, standard atomic weights of nitrogen and hydrogen | `cf-nitrogen` | ⚠ the **values are facts** and not copyrightable; the tabulation is IUPAC's — see below | two numbers only |

| 15 | **USDA NRCS Soil Data Access** (SSURGO), survey area ND031 — Foster County surface texture | `cf-acres` | U.S. Gov work, public domain (17 U.S.C. §105) | ✅ 8 texture classes (1 KiB) |
| 16 | **USDA-ARS** National Soil Dynamics Laboratory, *Tractor Power Requirement Recommendation Worksheet* | `cf-acres` | ⚠ **LAYERED** — federal tool, state data; **NOT DETERMINED** — see below | ✅ 8 chisel rows (1 KiB) |

## The four that need more than a row

### ⚠ Source 1 — the one whose terms are NOT settled

The Nebraska OECD tractor test reports are published by the **Nebraska Tractor
Test Laboratory at the University of Nebraska–Lincoln** — a *state* institution.
17 U.S.C. §105 removes copyright from works of the **federal** government and
says nothing about state ones, and Nebraska's public-records statute is about
**access**, not copyright. So "it is a public document" is an access claim being
mistaken for a licence.

✅ **The field now exists.** `cf-nebraska` predated the pattern that
`cf-electrolysis` introduced and `cf-storage` inherited; stage 4 touched the
crate, so the trigger fired and `Edition::terms` was added, carrying
`cf_nebraska::TERMS_NOT_DETERMINED`. `cf-tillage` references that constant
rather than restating the determination, so there is one copy to keep true.

⚠ **The field records the question, not an answer.** It says *not determined*,
and `every_edition_states_undetermined_terms` fails if it is ever made to read
as permission — the gate specifically rejects the phrase "public domain".
Nothing is redistributed either way: only transcribed figures, which are facts
and not the expression of them.

### ⚠ Source 5 — the only one that obliges us

NREL's `turbine-models` is BSD-3, so redistributing the power curve requires
reproducing the copyright notice, the conditions and the disclaimer. That is why this
repository has a **THIRD-PARTY DATA** section in `NOTICE`. It is the only entry in it.

### ⚠⚠ Source 16 — a federal tool over a state source

The worksheet is published by USDA-ARS, a federal laboratory, and would look
like a plain §105 work from its URL alone. It is not. Its own introduction says:

> "All implement recommendations and cost data incorporated into this tool are
> derived from the published **Mississippi State University 2026 Crop Planning
> Budgets**."

⛔ **The federal wrapper does not change where the numbers came from.** The data
is a *state* university's, which is the same §105-does-not-reach-it problem as
source 1, arriving by a route that hides it. `cf-acres` records the terms as
undetermined and `the_worksheets_terms_are_layered_and_say_so` fails if that row
is ever made to read as a plain federal work.

⚠ `cf-acres` also commits only the **implement rows**, not the cost columns —
the economics are outside the model and are the part most specific to one
state's budgets.

### ⚠ Source 14 — the one that is not a U.S. work at all

IUPAC is an international scientific union, not a U.S. federal body, so
17 U.S.C. §105 has nothing to say about it. `cf-nitrogen` transcribes **two**
numbers from it — the standard atomic weights of nitrogen and hydrogen — and
derives everything else (ammonia's molar mass, its nitrogen and hydrogen mass
fractions) by stoichiometry.

The determination is the same one this page already applies to the Nebraska
tables: **a standard atomic weight is a fact, and a fact is not the expression
of it.** What would be IUPAC's is the table, and the table is not reproduced.
`every_source_states_its_own_terms` fails if this row ever borrows the federal
reasoning it has no claim to.

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
in `cf-wind`, `Source` in `cf-electrolysis`, `cf-storage` and `cf-tillage`,
`Edition` in `cf-nebraska`.

Committed data additionally carries a SHA-256, and the two crates that carry it
check it **differently**, deliberately:

- `cf-wind`'s 206 KiB binary series is checked with `shasum` by hand, **after** a
  squash-merge rather than only before — the crate has no hashing dependency and
  says so.
- `cf-tillage`'s 21 KiB text extract is checked **in-process**, by
  `the_committed_extract_matches_its_digest`, because a digest no test computes
  is a 64-character string with no producer. That gate was added after a
  mutation flipped the constant and nothing went red.

⇒ If either file is edited, the digest beside it must be regenerated, and for
`cf-tillage` CI will say so without being asked.
