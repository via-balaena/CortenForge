# The tillage window: what was retrieved, and what is not there

`cf-tillage` commits its NASS extract rather than fetching it, because USDA
NASS crop progress is a U.S. federal work under 17 U.S.C. §105 with no
copyright. That is the opposite of stage 3's NIST isotherms, which are
copyright-asserted and so are retrieved and never committed. The distinction is
the point of `farm/PROVENANCE.md`.

This page records how the extract was produced, what was looked for and **not
found**, and the traps that cost time.

## The source

    https://www.nass.usda.gov/datasets/qs.crops_20260919.txt.gz   (1.05 GB)

Tab-separated, one row per published figure, 39 columns including
`SHORT_DESC`, `AGG_LEVEL_DESC`, `STATE_NAME`, `YEAR`, `WEEK_ENDING` and
`VALUE`.

⚠ **The Quick Stats API needs a key. The bulk file does not.** `GET
/api/get_param_values/` returns 401 without one; the dataset above is
unauthenticated. Use the bulk file.

## Reproducing the extract

```sh
curl -sL --fail "https://www.nass.usda.gov/datasets/qs.crops_<YYYYMMDD>.txt.gz" \
  | gunzip -c \
  | awk -F'\t' 'NR==1 || ($17=="NORTH DAKOTA" && $32=="WEEKLY")' \
  > nd_weekly.tsv
```

That yields 106,596 North Dakota weekly rows spanning 1981–2026. The committed
file is those rows reduced to three series, state level, weeks ending on or
after 25 August:

| code | NASS series |
|---|---|
| `DS` | `FIELDWORK - DAYS SUITABLE, MEASURED IN DAYS / WEEK` |
| `SOY` | `SOYBEANS - PROGRESS, MEASURED IN PCT HARVESTED` |
| `CORN` | `CORN - PROGRESS, MEASURED IN PCT HARVESTED` |

    921 rows   21,482 bytes
    sha256 99375f514fd18c09b53c8933e5bbfe71347bb64a767cd4b548b59c57e11d5615

The digest is gated by `the_committed_extract_matches_its_digest`, which
computes it in-process rather than trusting a number written beside the file.

## ⛔⛔ There is no fall-tillage series. This is checked, not assumed.

The obvious input for a tillage window would be a percent-complete series for
fall tillage. **It does not exist**, confirmed on two independent instruments:

1. **The bulk file.** Filtering all 106,596 North Dakota weekly rows for
   `tillage` or `fieldwork` in `SHORT_DESC` returns only the fieldwork
   days-suitable series and spring-start dates. No tillage progress, in any
   year from 1981 to 2026.
2. **The published PDFs.** The 2012 weekly reports for weeks ending 30 Sep,
   14 Oct, 28 Oct, 4 Nov and 18 Nov carry no tillage table.

⚠ **The word does appear in the prose**, and two of those five reports use it:

> "…in order to improve soil conditions for fall **tillage** and seeding of
> winter wheat." — week ending 30 September 2012

> "According to reporters, anhydrous application and fall **tillage** occurred
> in areas of the state with adequate levels of soil moisture." — week ending
> 14 October 2012

So the absence is of a **series**, not of the subject. An early grep hit only
the 28 October report, which has no mention at all, and nearly produced the
stronger and wrong claim.

**What replaces it is better.** `FIELDWORK - DAYS SUITABLE` is a count of days
the ground can actually be worked, which is what acres per season scales with.
A percent-complete series would have said how much tillage happened on the
state's real fleet; days suitable says how much *this* farm's one tractor
could do.

## The oracle check

The bulk file and the published PDFs are two renderings of the same figures, so
they can be compared. Over the five 2012 fall reports read:

| week ending | PDF prose | bulk file |
|---|---|---|
| 2012-09-30 | 6.8 | 6.8 |
| 2012-10-14 | 6.2 | 6.2 |
| 2012-10-28 | 4.2 | 4.2 |
| 2012-11-04 | 4.2 | 4.2 |
| 2012-11-18 | 4.1 | 4.1 |

Five for five, and **gated** by `the_committed_extract_reproduces_the_published_reports`
rather than left as prose — this is the only thing tying the committed extract
to NASS's own published reports, so it is the last claim on this page that
should have been unexecutable.

⚠ This validates **transcription**, not measurement: both come from the same
NASS field office, so an error in the survey is invisible to it.

## ⚠ Traps

**The filename rotates daily.** `qs.crops_20260912.txt.gz` was live on the 12th
and returns 404 on the 19th. The current name is linked from
`https://www.nass.usda.gov/datasets/` — read it rather than constructing it.

**2025 has a seven-week hole.** Fieldwork is published through 23 November, but
every week from 28 September to 16 November is missing — precisely the fall.
Summed naively the year reads as a catastrophic season (12.3 operable days
against a mean of 40.3) when the data is simply absent. `cf-tillage` rejects any
year publishing fewer than eight fieldwork weeks between 1 October and
25 November, and `an_incomplete_year_is_rejected_not_averaged` fires on 2025
specifically.

**Days suitable is state level only.** There is no agricultural-district or
county breakdown of this series — checked across all **1,024 days-suitable rows
of the intermediate `nd_weekly.tsv`** produced by the command above, every one
of which carries `AGG_LEVEL_DESC = STATE`.

⚠ That count is of the **intermediate** file, not of the committed extract. The
committed extract holds **404** days-suitable rows, because it keeps only weeks
ending on or after 25 August. Reproduce `nd_weekly.tsv` to check the claim;
counting the committed file will not reproduce it. The wind in `cf-wind` is
one grid point in Foster County; the fieldwork days are statewide. That
resolution mismatch is recorded in `UNMEASURED_HERE` rather than smoothed over.

**An empty window sums to negative zero.** Rust's `Sum for f64` folds from
`-0.0`, so a window containing no weeks returns `-0.0`, which prints as
`-0.00` and sorts below `+0.0` under `total_cmp`. Two real years — 2017 and
2023 — have a corn-gated window with no weeks in it. Every float sum over a
possibly-empty set in this crate uses `fold(0.0, …)` instead, gated by
`an_empty_window_is_positive_zero`.

## ⚠ What the window rule does, and does not, decide

Which rule opens the window **completely reorders the years**: 2012 is the 4th
worst fall of 19 on a fixed calendar and the 3rd best once the window may open
when harvest clears. But it moves the break-even engine efficiency by less than
half a percentage point, because a longer window buys operable days and
in-window hydrogen together.

Those are two different questions. An early version of this crate answered the
second with evidence from the first, and ranked the window rule as the **largest**
term in `BREAK_EVEN_CAVEATS` instead of the smallest. Both magnitudes are
measured by `the_inconsistent_comparison_inflates_the_window_rule`, which
reproduces the original mistake rather than quoting what it cost.
