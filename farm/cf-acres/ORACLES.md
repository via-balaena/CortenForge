# The two oracles behind the ceilings — and the one that is paywalled

`cf-acres` commits two extracts. This page is how to get them again, what the
retrieval traps were, and what the oracle recon found **closed**.

Both extracts are gated by `both_committed_extracts_match_their_digests`, which
computes the digests in-process.

    nd031_surface_texture.tsv   8 classes    sha256 012e99b4…a683fe2f7
    ars_chisel_plow.tsv         8 rows       sha256 e0989091…0964aa748

---

## 1. The soil — USDA NRCS Soil Data Access (SSURGO)

⚠ **This is a POST endpoint, not a fetchable URL.** `Source::url` names the
endpoint; without the query below it is unusable, which is the whole reason this
page exists.

```sh
curl -s -X POST "https://sdmdataaccess.nrcs.usda.gov/Tabular/post.rest" \
  -H "Content-Type: application/json" -d '{
  "format": "JSON+COLUMNNAME",
  "query": "SELECT mu.mukey, mu.muname, mu.muacres, c.compname, c.comppct_r,
            ch.hzdept_r, ch.hzdepb_r, ch.sandtotal_r, ch.silttotal_r,
            ch.claytotal_r, ch.dbthirdbar_r, cht.texcl
            FROM legend l
            INNER JOIN mapunit mu       ON mu.lkey = l.lkey
            INNER JOIN component c      ON c.mukey = mu.mukey
            INNER JOIN chorizon ch      ON ch.cokey = c.cokey
            INNER JOIN chtexturegrp chg ON chg.chkey = ch.chkey
            INNER JOIN chtexture cht    ON cht.chtgkey = chg.chtgkey
            WHERE l.areasymbol = '\''ND031'\''
              AND c.majcompflag = '\''Yes'\''
              AND ch.hzdept_r = 0
              AND chg.rvindicator = '\''Yes'\''
            ORDER BY mu.muacres DESC"
}'
```

Returns **256 rows**. Summed as `muacres × comppct_r / 100` per texture class
that gives the committed file: 8 classes, **286,446.74** component-weighted
acres, of which **233,888.91 is Loam — 81.6518 %**.

**`ND031` is Foster County, North Dakota** — the same county `cf-wind`'s WTK
grid point sits in. Confirm with:

```sh
… "query": "SELECT areasymbol, areaname FROM legend WHERE areaname LIKE '%Foster%'"
```

### ⚠ Retrieval traps

**The texture column is `texcl`, not `texdesc`.** The obvious name returns
`Invalid column name 'texdesc'` with HTTP **400** and an XML error body — which
a JSON parser reports as a parse failure rather than as the API's own message.
Read the body.

**`hzdept_r = 0` restricts to the surface horizon.** Measured: dropping it
returns **1,142 rows instead of 256** — every horizon of every component — and
the area weighting becomes meaningless.

**`rvindicator = 'Yes'` selects the representative texture.** Measured:
dropping it returns **287 rows instead of 256**. ⚠ That is **31 extra rows
across 256**, not a second texture for every component — a minority of them
carry one. An earlier draft of this page said "a component contributes several",
which overstates it; the numbers are here so the next reader does not have to
take either version on trust.

---

## 2. The implement — USDA-ARS Tractor Power Requirement Recommendation Worksheet

```sh
B="https://www.ars.usda.gov/ARSUserFiles/60100500/Worksheets"
curl -sL --fail "$B/Tractor%20Power%20Requirement%20Recommendation%20Worksheet.xlsx" -o worksheet.xlsx
curl -sL --fail "$B/Tractor%20Power%20Requirement%20Recommendation%20Worksheet%20Introduction.pdf" -o intro.pdf
```

The chisel plow rows are on **sheet 3**. The committed extract keeps five
columns — equipment, width, recommended power unit, hours per acre, acres per
hour — and **omits the cost columns** deliberately: economics are outside this
model, and they are the part most specific to one state's budgets.

### ⛔⛔ Its terms are layered, and the URL hides it

The introduction PDF says, in full:

> "All implement recommendations and cost data incorporated into this tool are
> derived from the published **Mississippi State University 2026 Crop Planning
> Budgets**. The information is offered exclusively for educational purposes."

So a `.gov` URL delivers **state** data. The federal wrapper does not launder
the provenance, `farm/PROVENANCE.md` records it as its own case, and a gate
fails if the source row ever reads as a plain 17 U.S.C. §105 work.

### ★★ It carries its own checksum

The worksheet prints **hours per acre** and **acres per hour** as separate
columns, so a mistranscribed rate stops being the reciprocal of its neighbour.
Eight rows, eight reciprocals, gated by `the_worksheet_reconciles_with_itself`.

### ⚠ And it is the weakest number in the crate

It anchors the time ceiling directly and it is **Mississippi-derived**. The
implement catalogue around it is bed hippers, listers and paratills on 30–40
inch rows — Delta row-crop practice, not eastern North Dakota. That is why the
rate is **swept rather than asserted**, and why the local soil texture is
committed beside it: a reader can see that this farm is 81.65 % loam and judge
the transfer themselves.

The anchor does survive one independent check: its implied ground speed at full
field efficiency is **4.52 mph**, against the **4.85 mph** `cf-nebraska`
measured for this tractor at 75 % drawbar load. Both are recomputed by
`the_anchor_implies_a_speed_the_tractor_could_actually_drive`.

---

## 3. ⛔ What the recon found closed

Three routes to a real draft model were checked in 2026-09 and all three are
shut. This is why `drawbar_kwh_per_acre` is a **lumped term** rather than a
draft equation.

| route | status |
|---|---|
| **ASABE D497** draft coefficients | **PAYWALLED** — sold through the ANSI webstore. The model's *structure* is open literature; the coefficient table is the standard's content and is not reproduced here. ⚠ A third-party upload of the full standard is in circulation; it is not used. |
| **Nebraska per-load wheel slip** | **ABSENT from every edition** — `cf_nebraska::ABSENT` records it as data. The individual OECD reports that would carry it are behind UNL DigitalCommons, which returned **403 to two different fetchers**. |
| **Nebraska drawbar surface** | **NOT STATED** in any edition read. ⛔ Asserting "concrete" from recall would prejudge the measured-to-field conversion, which is exactly the term being modelled. |

⚠ I also searched Ag Data Commons for a public-domain measured-draft dataset and
found none — **one search, so a hypothesis rather than a confirmed absence.**

If D497 is ever obtained, it slots in as a **retrieved oracle** the way
`cf-storage` uses NIST: validate against it, commit the measurement and the
command, never the table.
