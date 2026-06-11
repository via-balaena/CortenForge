# CF-CAST Bonded Cast-in-Place + Shell Texturing Recon

> **Status:** RECON SCAFFOLD (pre-implementation). Cold-read pass-1 at end (§9).
> **Date:** 2026-06-11
> **Trigger:** Cendrillon design session while wiring the interior-ridge (canal)
> toggle + the "parts to generate" picker. The user (product owner) pulled two
> ideas together that reshape the casting model:
> 1. **Texture the *outside* of the shells, not just the layer-0 plug's interior
>    canal** — so a device can carry exterior ridges (e.g. for a different
>    anatomy/gender), applied to *every* selected shell layer for even layering.
> 2. **One plug, cast-in-place** — you don't need a plug per layer; the cured
>    layer N *is* the plug for layer N+1. Keep the original plug, pour each new
>    layer onto the previous cured one so the silicone bonds as it cures.
> **The synthesis:** the inter-layer ridges are not cosmetic — they are the
> *mechanical key* that makes single-plug bonded casting robust against a weak
> silicone-to-silicone chemical bond across durometers.
> **Direction (2026-06-11):** SDK keeps BOTH casting models (detachable +
> bonded); **Cendrillon defaults to bonded**. Flagship-vs-product split per
> `MISSION.md`.
> **Composes on:** branch `feat/cendrillon-ridges-and-part-selector` @ `f182908f`
> (the interior-ridge toggle + per-piece `export_selected` already landed there;
> `main` = `e75cb2fc`). Builds directly on the canal arc
> ([[CF_CAST_CANAL_INTERIOR_RECON]]).

---

## 1. The two casting models

### Detachable (today, SDK default — unchanged)
Per layer N: a plug `plug_layer_N` (layer 0 = scan-derived; N>0 = `layers[N-1].body`)
+ 2 cup halves. Each layer is cast **independently** against its own plug; the
cured shells **nest** at assembly and can be pulled apart for cleaning/replacement.
`export_molds_v2` ships `3L` STLs (`2L` cups + `L` plugs).

### Bonded cast-in-place (new — Cendrillon default)
**One plug** (`plug_layer_0` only) + 2 cup halves per layer. Workflow:
1. Plug + cup 0 → pour layer 0 → cure → remove cup 0, **leave layer 0 on the plug**.
2. Cup 1 around (plug + cured layer 0) → pour layer 1 *onto* layer 0 → cure → remove cup 1.
3. Repeat to the outermost layer.

Output: `2L` cups + **1** plug. One growing, bonded part (not detachable shells).

**Why the extra plugs are redundant in bonded mode:** `plug_layer_{N>0}` is just
`layers[N-1].body` — the outer geometry of the previous shell. Cast-in-place uses
the *real* cured layer N-1 as that plug, so the printed copy is unnecessary.

> NB: the per-piece part picker (`export_selected`, just landed) already lets a
> user generate only `plug_layer_0`. Bonded mode makes that **semantic** (the
> default + the procedure), and lets the SDK *skip computing* the redundant plugs.

---

## 2. Shell texturing (the new geometry)

The canal feature (`CanalSpec` + `CanalFeatureSdf`, [[CF_CAST_CANAL_INTERIOR_RECON]])
is a periodic displacement field evaluated in a cylindrical frame around the
centerline, added to a base solid's SDF: `eval(p) = base.evaluate(p) + inset_field(...)`.
It currently wraps **the layer-0 plug**. The field wraps **any** `Solid`, so the
same machinery applies to a **layer body's outer surface**:

- **Interior ridges (today):** field on the layer-0 plug → ridges on the *inside*
  of the cured device (the canal). Plug pulls straight out → undercuts harmless.
- **Exterior / inter-layer ridges (new):** field on each selected layer *body* →
  carved into that layer's **cup cavity** → ridges on the *outer* surface of that
  cured layer. For inner layers this surface becomes the **interface** to the next
  layer (mechanical interlock + even nesting); for the outermost layer it is the
  **device exterior**.

Applied to **every selected shell layer** (user's call: "every layer's shell, for
layer even-ness") so each pour keys into the one below with a matching pattern.

### Geometry note
A layer body is used in two roles in *detachable* mode (cup cavity for layer N
AND plug for layer N+1), so texturing it there has two effects. In **bonded**
mode only `plug_layer_0` exists, so a body's texture only affects its own cup
cavity — cleaner. This is another reason texturing pairs naturally with bonded.

---

## 3. Risks / physical caveats (must design for)

- **R1 — silicone-to-silicone bond across durometers.** Platinum-cure silicone
  self-bonds uncured-on-cured, but it's finicky (cure inhibition, surface
  cleanliness, working window). **Mitigation = the inter-layer ridges**
  (mechanical retention even if the chemical bond is weak). This is the whole
  point of texturing every interface. *Open:* do we also recommend a tie-coat /
  surface prep in the procedure?
- **R2 — demolding undercuts.** A ridged cup cavity is an undercut; at each step
  a *rigid* printed cup is peeled off a *flexible* cured layer. Shallow/rounded
  ridges release (silicone flexes); aggressive annular ridges won't. **Defaults
  must be demoldable** — bound ridge depth, prefer rounded profiles, consider
  ridge orientation vs the cup-split axis. Needs a spike (§7 S0).
- **R3 — layer thickness vs ridge depth.** Inter-layer ridges displace into the
  next layer's thickness; a deep ridge on a thin layer could breach it. Gate
  ridge depth against layer thickness (cf. the canal's existing
  `max_inward_depth_m` ⊂ wall check).

---

## 4. Architecture (proposed)

### SDK (cf-cast)
- **`CastMode { Detachable, Bonded }`** on `CastSpec` (or a build option).
  `Detachable` = today (bit-preserved). `Bonded` = emit only `plug_layer_0`;
  per-layer plugs are skipped at the source (not just deselected).
- **Shell texture**: a per-layer outer-surface texture spec. Reuse `CanalSpec`'s
  field (`inset_field` / `CanalFeatureSdf`) applied to `layer.body` before the
  cup composition. New `ShellTextureSpec` (or a reused `CanalSpec` with an
  "exterior" sign) per layer; threads through `compose_piece_shared`.
- **Procedure**: bonded `write_procedure_v2` variant — cast-in-place / bond-as-
  you-go instructions (vs today's detachable-shell assembly prose).

### Cendrillon (cf-studio)
- **New optional wizard page "Texture"** *after* Design, *before* Make molds:
  `1 Scan → 2 Clean → 3 Design → 4 Texture (optional, skippable) → 5 Make molds
  → 6 Print → 7 Pour`. (Exterior ridges need the layer stack, so it must follow
  Design.) Step enum + `Project` state machine + nav + renumber.
- The page hosts **both**: interior ridges (canal, layer-0 plug — moved off the
  step-4 panel built on this branch) **and** exterior/shell ridges (per selected
  shell), reusing the existing ridge controls.
- **Default Cendrillon to `CastMode::Bonded`** (single plug + bonded procedure).

---

## 5. Blast radius / what stays bit-identical
- `CastMode::Detachable` + no shell texture = today's `export_molds_v2`, byte-
  identical (the SDK default path is untouched, same discipline as the
  `export_selected` landing).
- The interior canal is unchanged geometry; it just *moves* in the Cendrillon UI
  (step-4 panel → the new Texture page).

---

## 6. Open questions
- **Q1** Exterior ridge profile/sign: grooves (inward) vs bumps (outward)? Default
  depth bound for demoldability (R2)?
- **Q2** Procedure: recommend a tie-coat / IPA wipe between layers, or rely on
  ridges alone (R1)?
- **Q3** Should the Texture page expose per-layer texture, or one shared shell
  pattern applied to all selected layers (user leaned "every layer, for evenness"
  → one shared pattern is simpler and matches intent)?
- **Q4** Does bonded mode change the pour plan / cure timing (inter-layer bond
  window)?

---

## 7. Slice plan (proposed)

- **S0 — spike (de-risk R2/R3):** apply the canal field to one layer body on
  `base_mold`, mesh the cup, eyeball the cavity undercut + measure ridge depth vs
  demoldability. Bounds the default depth/profile. *No ship.*
- **S1 — SDK `CastMode::Bonded`:** emit only `plug_layer_0`; detachable unchanged
  + byte-identical. Tests + the single-plug output assertion.
- **S2 — SDK bonded procedure:** cast-in-place `write_procedure_v2` variant.
- **S3 — SDK shell texture:** `ShellTextureSpec` on layer bodies via the canal
  field; depth⊂thickness gate (R3); demoldable defaults from S0.
- **S4 — Cendrillon Texture page:** new Step + state machine + nav + move the
  interior-ridge panel here; add exterior/shell ridge controls.
- **S5 — Cendrillon bonded default:** wire `CastMode::Bonded` + bonded procedure;
  parts picker reflects the single plug.

Each slice: recon→(spike)→sliced PR→n+1 cold-read→pre-PR local ultra-review, per
[[feedback-pre-pr-local-ultra-review]] + [[feedback-head-engineer-owns-technical-calls]].

---

## 8. Decisions banked (this session)
- SDK keeps **both** casting models; **Cendrillon defaults to bonded**.
- Exterior ridges applied to **every selected shell layer** (evenness).
- Inter-layer ridges are the **mechanical interlock** that de-risks the bond.
- New **optional Texture page after Design**.

## 9. Cold-read pass-1
_(pending — fill after first read-through)_
