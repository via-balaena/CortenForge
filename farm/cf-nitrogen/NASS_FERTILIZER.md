# The nitrogen rate: what was retrieved, and what it is a rate of

`cf-nitrogen` commits its NASS extract, because USDA NASS is a U.S. federal work
under 17 U.S.C. §105 with no copyright — the same reasoning as
`cf-tillage/NASS_VALIDATION.md`, and the opposite of stage 3's NIST isotherms.

## The source

    https://www.nass.usda.gov/datasets/qs.environmental_20260919.txt.gz   (72 MB)

⚠ **Fertilizer is in the ENVIRONMENTAL dataset, not CROPS.** Crop progress and
fieldwork are in `qs.crops`; the Agricultural Chemical Use Program — which is
where fertilizer rates live — is filed under ENVIRONMENTAL. Looking in the
1.05 GB crops file for a nitrogen rate finds nothing, and finding nothing there
is not evidence that NASS does not publish one.

⚠ **It is not in `SHORT_DESC` either.** Zero of the 23,947 North Dakota
environmental rows mention "nitrogen" or "fertilizer" in that column — that is
the `nd_env.tsv` the command below produces, not the committed extract. The
nutrient is in the **domain** dimension:

    DOMAIN_DESC    = FERTILIZER
    DOMAINCAT_DESC = FERTILIZER: (NITROGEN)
    SHORT_DESC     = CORN - APPLICATIONS, MEASURED IN LB / ACRE / YEAR, AVG

## Reproducing the extract

```sh
curl -sL --fail "https://www.nass.usda.gov/datasets/qs.environmental_<YYYYMMDD>.txt.gz" \
  | gunzip -c \
  | awk -F'\t' 'NR==1 || $17=="NORTH DAKOTA"' \
  > nd_env.tsv
```

23,947 rows. The committed file is those reduced to the nitrogen domain and the
four per-acre measures:

    356 rows   22,452 bytes
    sha256 b589e55b320f6eb3c3ab76a8978e2929919b0db16a3e5265bcaf50d5eddb93b3

Gated by `the_committed_extract_matches_its_digest`, computed in-process.

## ⛔⛔ Two denominators, and they are not the same

NASS prints this quantity **four ways**, and two of them are per acre:

| measure | unit | denominator |
|---|---|---|
| `PER_APP` | `LB / ACRE / APPLICATION, AVG` | a treated acre, one pass |
| `APPS` | `NUMBER, AVG` | passes per year |
| `PER_YEAR` | `LB / ACRE / YEAR, AVG` | a **treated** acre, whole year |
| `PCT_AREA` | `PCT OF AREA PLANTED, AVG` | share of **planted** area treated at all |

**`PER_YEAR` is per treated acre, not per planted acre.** In 1990 only **80%**
of North Dakota corn area was treated, so nitrogen per planted acre was a fifth
below the printed rate. By 2021 the treated share is **99%** and the gap has
almost closed — but it is a real denominator difference and the crate carries
`PctOfAreaPlanted` so it stays visible.

## ★★ The survey carries its own checksum

`PER_APP × APPS` must reproduce `PER_YEAR`. Checked by **rounding-interval
overlap**, not a tolerance: NASS prints the count to one decimal and the rates
to whole pounds, so `65 × 1.9` must be allowed to mean anything in
`[64.5, 65.5] × [1.85, 1.95]`.

    86 reconcile
     2 disagree   — North Dakota corn, 2001 and 2003
     1 indeterminate — organic spring wheat 2009, entirely withheld

⚠ A first pass using a fixed ±1 lb tolerance reported **12** mismatches. Eleven
of them were the rounding of `APPS`, not disagreements. The interval rule is
what separates the two.

⛔ The two real disagreements are **recorded, not repaired**. Both miss by about
a pound. Why is **NOT ISOLATED** — a revision between printings, a suppressed
application class, or rounding on a figure this extract cannot see would all
look like this. Nudging any of the three to close it would be authoring an input
from the check that validates it.

## ⛔⛔ The production practice is part of the key

`PRODN_PRACTICE_DESC` separates `ALL PRODUCTION PRACTICES` from `ORGANIC`.

**Dropping it collapsed two distinct 2009 spring-wheat records into one**, and
`records()` silently returned whichever the search reached first. It was caught
because the four withheld cells all belonged to the record that was being
overwritten.

This is not only a key bug. An **organic acre receives no synthetic anhydrous
ammonia**, so folding it into a Haber-Bosch hydrogen model counts acres that
need no Haber-Bosch hydrogen. The organic record is carried so it is visible and
excluded on purpose, gated by `organic_is_carried_and_excluded`.

## ⚠ `(D)` is withheld, not missing

NASS suppresses a cell when publishing it would disclose an individual
operation. The quantity **exists and the survey knows it**; only the publication
is withheld. `Cell::Withheld` keeps that distinct from "not surveyed", and
`a_withheld_cell_is_not_a_missing_one` fails if a withheld cell ever reads as
zero.

All four withheld cells in this extract are the organic spring-wheat record.

## ⛔ There is no application-timing series

Checked the same way the tillage window was: across the **106,596** North Dakota
weekly rows of the intermediate file, **zero** mention fertilizer, anhydrous or
application.

⚠ **That file is not committed.** It is the `nd_weekly.tsv` produced by the
command in `cf-tillage/NASS_VALIDATION.md`; this crate commits 356 fertilizer
rows and `cf-tillage` commits 921 fall-week rows, and neither is the collection
the absence was checked against. Reproduce it before relying on the claim.

### The second instrument

The published PDFs, checked here rather than taken from anywhere. The five 2012
fall reports — weeks ending 30 Sep, 14 Oct, 28 Oct, 4 Nov and 18 Nov. The file
name is `cw-MMDD` for the week-ending date:

```sh
BASE="https://www.nass.usda.gov/Statistics_by_State/North_Dakota/Publications/Crop_Progress_&_Condition/2012"
for f in cw-1001 cw-1015 cw-1029 cw-1105 cw-1119; do
  curl -sL --fail "$BASE/$f.pdf" -o "$f.pdf"
  pdftotext -layout "$f.pdf" "$f.txt"
  grep -ic "fertiliz\|anhydrous" "$f.txt"
done
```

| report | week ending | mention | where |
|---|---|---|---|
| cw-1001 | 30 Sep | — | — |
| cw-1015 | 14 Oct | "anhydrous application and fall tillage occurred" | narrative prose |
| cw-1029 | 28 Oct | — | — |
| cw-1105 | 4 Nov | "Fertilizer application progressed in areas where moisture…" | narrative prose |
| cw-1119 | 18 Nov | "…included applying fertilizer and moving cattle" | narrative prose |

**Three mentions across five reports, zero table rows.**

⚠ The five week-endings are the ones `cf-tillage`'s committed extract carries
as days-suitable rows (6.8, 6.2, 4.2, 4.2, 4.1), so the *dates* are checkable in
this repository even though the PDFs are not committed.

⚠ Two traps in getting that right. The reports are **two-column**, and
`pdftotext -layout` merges the left column's prose with the right column's table
onto one line — an unsplit check reported the anhydrous sentence as a table row
carrying four numbers, which actually belonged to the Pasture Condition table
beside it. And a "zero table rows" result means nothing unless the classifier
can see a table row at all: the control finds three real ones (`Pasture and
range`, 4 numeric columns) in the same files.

⛔ An earlier version of this page attributed this finding to
`cf-tillage/NASS_VALIDATION.md`. That page records no such thing — it documents
that these PDFs carry no *tillage* table, and does not mention fertilizer.

What exists is the sentence the window is taken from — week ending 14 October
2012:

> "According to reporters, **anhydrous application** and fall tillage occurred
> in areas of the state with adequate levels of soil moisture."

⇒ Fall nitrogen is drawn on the operable-days window `cf-tillage` measures,
because both need ground a machine can drive on and NASS reports them together.
That is a **modelling decision with a citation**, not a measurement, and
`UNMEASURED_HERE` records what would replace it — including the fall/spring
split, which this crate does not attempt.

## ⚠ North Dakota is not the Corn Belt

This chain's own research context puts corn nitrogen at 150–200 lb N/acre and
attributes it to "USDA NASS / state extension rates". The same document states
the nitrogen leg is "roughly 6–8× the fuel leg" and that hydrogen is "17.6% of
NH3 by mass". Read from the NASS survey itself:

| year | lb N/acre/yr |
|---|---|
| 1990 | 69 |
| 2000 | 98 |
| 2010 | 160 |
| 2016 | 133 |
| 2018 | 146 |
| 2021 | **118** |

No surveyed year reaches 200, and the most recent reading is 118. Using the
circulated range would overstate North Dakota's nitrogen leg by 27% to 69%.
