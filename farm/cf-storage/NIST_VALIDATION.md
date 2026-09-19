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

Four isotherms, normal hydrogen (CAS 1333-74-0, `ID=C1333740`), 1–701 bar in 10 bar steps:

```sh
out=$(mktemp -d)
for T in 250 273.15 300 330; do
  curl -sS -o "$out/nist_T$T.tsv" \
    "https://webbook.nist.gov/cgi/fluid.cgi?Action=Data&Wide=on&ID=C1333740&Type=IsoTherm\
&Digits=8&PLow=1&PHigh=701&PInc=10&T=$T&RefState=DEF&TUnit=K&PUnit=bar&DUnit=kg%2Fm3\
&HUnit=kJ%2Fkg&WUnit=m%2Fs&VisUnit=uPa*s&STUnit=N%2Fm"
done

CF_NIST_H2_ISOTHERMS="$out" cargo test -p cf-storage --test storage -- --ignored --nocapture
rm -rf "$out"          # ⛔ do not leave these in the tree
```

## ⛔⛔ Three retrieval traps, all measured on this endpoint

1. **`PInc` is a request, not a contract.** Asking for 1 bar steps over 1–901 bar
   returns **1.5 bar** steps — the CGI caps the response at roughly 600 rows and
   silently rescales. The test asserts the returned grid *is* the requested grid.
2. **Rows are duplicated at a phase-label boundary.** The 1–701 bar isotherm returns
   **73 rows for 71 requested**: the 21 bar point appears three times as the `Phase`
   column flips from `vapor` to `supercritical`. Keying by row position would shift
   every figure above 21 bar. The parser keys by the printed pressure.
3. **The file self-validates, so check it before trusting it.** Every row satisfies
   `ρ·v = 1` and `U + Pv = H` to about 1 part in 10⁸. The test asserts both before it
   compares anything.

## What was retrieved, and what it means if these change

Measured 2026-09-18. A checksum mismatch here does **not** mean the retrieval failed —
it means **NIST revised the data**, and the committed error figures were measured
against the retrieval below.

| file | bytes | SHA-256 |
|---|---|---|
| `nist_T250.tsv`    | 11139 | `6122d89fa8b17cd6b733985145184f0108ae8e5e6e3b44945575788c5f8c4d71` |
| `nist_T273.15.tsv` | 11137 | `18ac98d1e0769f59ce31ea23e993b568013f667ab755860965c433ee1521c30b` |
| `nist_T300.tsv`    | 11136 | `58b9d6bba58fa3302a7bb07ccaf909e344718384779553a4d338cbb1e4cd83b5` |
| `nist_T330.tsv`    | 11135 | `4523db4faf36bac0ca9967ba0ca34170cd2c7640d1b3350d5ab11bbdae7c23f6` |

## The measurement

284 states, 250–330 K, 1–701 bar:

| model | worst density error | at 350 bar, 300 K | at 350 bar, 273 K |
|---|---|---|---|
| ideal gas | **53.43 %** | +22.03 % | +23.88 % |
| second-virial covolume | **6.66 %** | +1.49 % | +2.20 % |

★ The second row is what one parameter from a 1964 paper buys. The 273 K column is the
one this farm runs at — see `CARRINGTON_FALL`, measured from NOAA rather than adopted
from the source document's 300 K convention.

⚠ The residual grows with pressure and shrinks with temperature, which is the signature
of the **third** virial coefficient this model drops. That is checked by its shape —
`residual/ρ²` stays within a factor of three along each isotherm — not by fitting a
value. Restoring the term would roughly halve the error at 700 bar, and the chain's
headline cannot see the difference, so it is not in the model.
