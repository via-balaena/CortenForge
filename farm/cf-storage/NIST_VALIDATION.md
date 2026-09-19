# NIST hydrogen isotherms — a retrieved oracle, never a committed asset

This crate's equation of state is measured against the **NIST Chemistry WebBook**.
Those reference values are **not in this repository and must not be added to it**.
What is committed is the *measurement* — the error figures in
[`NIST_DENSITY_COMPARISON`](src/lib.rs) — and this page, which says how to reproduce it.

This is the same shape as `design/cf-fsu-geometry/BODYPARTS3D.md`: fetch to a temp
path, point an environment variable at it, run an `#[ignore]`d test. ⚠ Copy the
structure, not the reason. There the data is CC BY-SA; here it is copyright-asserted
by a U.S. federal agency, which is the more surprising case.

## Why this one is not public domain

Most U.S. Government works carry no copyright (17 U.S.C. §105), and the other two
sources this crate reads — DOE Program Record 9013 and a 1964 NBS paper — are exactly
that. **NIST Standard Reference Data is the exception**, under the Standard Reference
Data Act (15 U.S.C. §290e):

> © 2026 by the U.S. Secretary of Commerce on behalf of the United States of America.
> All rights reserved.

⇒ *"It is a government document, therefore it is free"* is not a determination. This
crate records the terms of every source in `Source::terms` because of this page.

## What the model is measured against — and what it is NOT

⛔ The oracle is **not independent of Record 9013**. That record states its theoretical
figures were *"determined from exergy differences using the standard properties of
'normal' hydrogen … as reported by NIST"*. Agreeing with both is **one** check.

The independent leg is the model's only parameter, B(T), which comes from
**Goodwin, Diller, Roder & Weber, *Second and Third Virial Coefficients for Hydrogen*,
J. Res. NBS 68A(1), 1964, pp. 121–126, [doi:10.6028/jres.068A.011](https://doi.org/10.6028/jres.068A.011)** —
public domain, and used by neither the record nor the WebBook comparison. Nothing NIST
publishes enters the model, which is the only reason this comparison can fail.

⚠ Not *fully* independent: NIST's current equation of state was fitted to experimental
PVT data overlapping what the 1964 paper reduced. Two reductions of overlapping
measurements forty-five years apart, not two unrelated observations.

## Fetch

Four isotherms, normal hydrogen (CAS 1333-74-0, `ID=C1333740`), **20–880 bar in 10 bar
steps**. ⚠ The lower bound is 20 rather than 1 deliberately: it is the record's own
inlet, and starting there puts **350, 440, 700 and 880 bar on the grid**. An earlier
grid of `1 + 10k` contained none of them, so every constant named `_at_350_bar_` was in
fact measured at 351.

```sh
out=$(mktemp -d)
for T in 250 273.15 300 330; do
  curl -sS -o "$out/nist_T$T.tsv" \
    "https://webbook.nist.gov/cgi/fluid.cgi?Action=Data&Wide=on&ID=C1333740&Type=IsoTherm\
&Digits=8&PLow=20&PHigh=880&PInc=10&T=$T&RefState=DEF&TUnit=K&PUnit=bar&DUnit=kg%2Fm3\
&HUnit=kJ%2Fkg&WUnit=m%2Fs&VisUnit=uPa*s&STUnit=N%2Fm"
done

CF_NIST_H2_ISOTHERMS="$out" cargo test -p cf-storage --test storage -- --ignored --nocapture
rm -rf "$out"          # ⛔ do not leave these in the tree
```

## ⛔⛔ Three retrieval traps, all measured on this endpoint

1. **`PInc` is a request, not a contract.** Asking for 1 bar steps over 1–901 bar
   returns **1.5 bar** steps — the CGI caps the response at roughly 600 rows and
   silently rescales. The test asserts the returned grid *is* the requested grid.
2. **Rows are duplicated at a phase-label boundary.** A 1–701 bar isotherm returns
   **73 rows for 71 requested**: the 21 bar point appears three times as the `Phase`
   column flips from `vapor` to `supercritical`. Keying by row position would shift
   every figure above 21 bar.
   ⚠ **The grid above does not trigger this** — it starts at 20 bar, past the
   boundary, and returns 87 distinct rows for 87. The parser still keys by the
   printed pressure, because avoiding a trap by accident is not the same as being
   immune to it.
3. **The file self-validates, so check it before trusting it.** Every row satisfies
   `ρ·v = 1` and `U + Pv = H` to about 1 part in 10⁸. The test asserts both before it
   compares anything.

## What was retrieved, and what it means if these change

Measured 2026-09-18. A checksum mismatch here does **not** mean the retrieval failed —
it means **NIST revised the data**, and the committed error figures were measured
against the retrieval below.

| file | bytes | SHA-256 |
|---|---|---|
| `nist_T250.tsv`    | 13265 | `7c0fb1148d4a73b3a6b439f0004838b1df8fc1ede98fc600ad48ffbe4dbb1bf1` |
| `nist_T273.15.tsv` | 13264 | `70067ec7b6bd572bf42cd942461485745c10f0318f031bc173216e43db46bb82` |
| `nist_T300.tsv`    | 13263 | `17f0de9759b9adb44bb29a4ea5f80b94f3fabeffeb3416ab3099cb72ffc703fb` |
| `nist_T330.tsv`    | 13262 | `ace4d0861b02c598fd02cf6209ca05a8ff74d77e298170128f803412b3e41d57` |

## The measurement

348 states, 250–330 K, 20–880 bar:

| model | worst | 350 bar, 300 K | 350 bar, 273 K | 700 bar, 300 K | 700 bar, 273 K |
|---|---|---|---|---|---|
| ideal gas | **67.44 %** | +21.96 % | +23.80 % | +44.87 % | +49.06 % |
| second-virial covolume | **8.00 %** | +1.48 % | +2.19 % | +3.21 % | +4.75 % |

★ The second row is what one parameter from a 1964 paper buys. The 273 K columns are
the ones this farm runs at — see `CARRINGTON_FALL`, measured from NOAA rather than
adopted from the source document's 300 K convention.

⚠ The worst case is now at **880 bar**, the refuelling overpressure Record 9013
discusses, rather than at the old grid's 701 bar ceiling. Nothing about the model
changed; the comparison simply reaches further.

★ **A second thing the retrieval buys.** Because 350, 440, 700 and 880 bar are on the
grid, the same isotherm checks the record's own Table 1 against the equation of state
it says it used: two of its four theoretical figures agree, and the 700 and 880 bar
ones sit **1.30 %** and **1.08 %** low. `RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT`
carried that claim with no producer at all until this gate existed.

⛔ That is **not** an independent check of the record — Record 9013 says its figures
came from NIST, so it compares the record with a later version of its own source.

⚠ The residual grows with pressure and shrinks with temperature, which is the signature
of the **third** virial coefficient this model drops. That is checked by its shape —
`residual/ρ²` stays within a factor of three along each isotherm — not by fitting a
value. Restoring the term would roughly halve the error at 700 bar, and the chain's
headline cannot see the difference, so it is not in the model.
