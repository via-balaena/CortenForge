# CF-CAST Scan-Surface Texture + Plug-Shaping Reorg Recon

> **Status:** RECON (pre-implementation). 2026-06-11.
> **Trigger:** Cendrillon design session. The user reframed the texture model:
> texture should be applied to the **cleaned scan surface itself** (the real
> device surface), not to a separate capsule proxy and not to the plug/shells
> independently.
> **Composes on:** branch `feat/cendrillon-ridges-and-part-selector` @ `34f75ccb`.
> Supersedes the interior/exterior split (`6ec05763`) and the capsule preview.

---

## 1. The insight

The cleaned scan **is** the device surface. In the cast:
- `plug = scan.offset(−cavity_inset)` (the core, offset inward),
- `layer_body[N] = scan.offset(+cumulative_thickness − inset)` (the shells, outward).

So if the ridge field is composed onto the **scan surface once**, every derived
surface (plug + every shell) carries the *identical* displacement through its
offset. Same displacement on all surfaces ⇒ the gaps between them — the **wall
thicknesses — stay constant**. One edit surface, propagated everywhere, steady
wall. (The field is a function of the canal-frame coords `(frac, axial, θ)`, not
of radius, so it adds the same amount to every offset surface.)

**All** features propagate cleanly (rings, fine texture, side-pinch, tip-relief,
orientation) — there's no geometric reason to drop the one-sided ones; they ride
the offsets too and keep the wall steady. So: ONE unified ridge set, no
interior/exterior split.

## 2. The inset resolution

The texture lives on the scan, but the **cavity inset** is what turns the scan
into the plug — so the previewed piece isn't final until the inset is fixed.
**Resolution (user's option c):** move the cavity inset onto the texture page.
That page becomes **"Shape your piece"** — set the snugness (inset) + the ridges,
previewed on the **real cleaned scan** offset inward by that inset (the actual
plug). The old Design step becomes purely the **layer stack**, built outward off
the shaped plug.

## 3. New wizard flow (7 steps, from 8)

`1 Add scan → 2 Clean → 3 Shape your piece (inset + ridges, REAL-scan preview)
→ 4 Choose how it feels (the layer stack) → 5 Make molds → 6 Print → 7 Pour`

> **Update (shipped + post-ultra-review, 2026-06-11):** implemented on branch
> `feat/cendrillon-ridges-and-part-selector`, then hardened by a local
> ultra-review. Two corrections to the plan below: (1) wall-preservation is NOT
> automatic — `build_canal_plug` frames `frac` off each body's own AABB, so the
> plug + bodies must share ONE frame (`build_canal_plug_framed`, framed off the
> plug span) or the rings drift apart; pinned by the cross-body wall test in
> `cf-cast`. (2) The old `shell_texture` R3 depth-gate was **dropped** (not
> kept): inward features only add wall, and the one OUTWARD feature (suction
> bulge) is now covered by a new cup-wall gate in `derive.rs`. The textured cup
> pieces + N>0 plugs also need the canal-field skin (`plug_layer_0_field_skin_m`,
> now the shared canal skin) so the narrow-band mesher doesn't drop ring geometry.

## 4. Implementation

- **cf-cast-cli (`derive.rs`):** apply the (one) canal/texture field to the plug
  **AND every layer body** with the same spec — currently it's plug-only +
  separate `shell_texture` on bodies. **Unify:** the `canal` config IS the
  texture; drop `shell_texture` (subsumed). Frame all bodies off ONE shared span
  (the plug's) so the rings line up and the wall stays constant (the old R3 gate
  is replaced by a suction cup-wall gate). Bodies that carry the texture mesh
  with the canal-field skin; cup-cell texture is coarser than the plug cell.
- **cf-studio-core:** Step reorg — drop `InteriorTexture`/`ExteriorTexture`; add
  `ShapePiece` (before `DesignLayers`). New `PlugDraft { cavity_inset_m, ridges:
  RidgeOptions }` artifact; `DesignDraft` keeps `{ cavity_inset_m, layers }` as
  the engine/`design.toml` type (GUI reconstructs it from plug.inset + layers).
  Drop `ShellRidgeOptions` + `TextureDraft`. Schema v3→v4 + migration.
- **cf-studio-engine:** `generate_molds_for_design` takes the inset + layers +
  one `RidgeOptions` (→ canal config). **Real-scan plug preview:** load the
  cleaned scan SDF once (cache), then on each edit offset by the inset + compose
  the ridge field + coarse marching cubes → the real plug mesh. Debounced /
  background (heavier than the capsule).
- **cf-studio-gui:** the "Shape your piece" page = cavity-inset spin + the ridge
  editor + the real-scan preview; the "Choose how it feels" page = the layer
  stack only. Renumber. One ridge editor (the interior/exterior split is gone).

## 5. Risks / notes
- **Texture survival through the offset** — the field is added pre-offset; the
  offset shifts the iso-surface uniformly, so the displacement is preserved
  (steady wall). Fine sinusoidal texture on the *outermost* shell at coarse cup
  cells may smear — gate cell size as the canal plug already does.
- **Preview cost** — loading the scan SDF (flood-fill) is the expensive part; do
  it once per page-entry and cache, re-mesh on edits.
- **Blast radius** — the detachable + bonded paths are unaffected (texture is
  still optional / off by default). This reorganizes WHERE texture is applied +
  the wizard, not the export pipelines.

## 6. Slices
- **S1** cf-cast-cli: unify texture onto plug + bodies; drop `shell_texture`.
- **S2** cf-studio-core: Step reorg + `PlugDraft`; drop shell/texture types.
- **S3** cf-studio-engine: rewire `generate_molds_for_design`; real-scan preview.
- **S4** cf-studio-gui: "Shape your piece" + "Choose how it feels" pages.
