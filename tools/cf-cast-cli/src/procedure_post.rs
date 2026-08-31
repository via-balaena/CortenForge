//! Slices 9.6c + 9.5 — post-process `cf-cast`'s `procedure.md` to
//! surface design-level metadata cf-cast itself doesn't see.
//!
//! `cf-cast::write_procedure_v2` runs against [`cf_cast::CastSpec`],
//! which is the workshop-level geometry contract — it doesn't know
//! about `cf-device-design`'s `cavity.inset_m` (the press-fit
//! reservation, slice 9.6c) or per-layer `slacker_fraction` (slice
//! 9.5). Both flow from the `.design.toml` lift but stop at the
//! cf-cast-cli boundary; the procedure markdown is the natural place
//! to surface them for the workshop user.
//!
//! cf-cast-cli knows both values and is the right place to weave them
//! into the procedure markdown without expanding cf-cast's public
//! surface. So we post-process: read the procedure file back, splice
//! new sections in above stable cf-cast anchor headers, write the
//! patched file back. No-op when the relevant value is absent (inset
//! == 0, no layer has slacker > 0).

use std::path::Path;

use anyhow::{Context, Result, bail};

/// The h2-header anchor `cf-cast`'s `generate_procedure_markdown_v2`
/// emits via `write_materials_table` — see
/// `design/cf-cast/src/procedure.rs:100` and `:352`. We inject the
/// `## Press-Fit Reservation` section above this line so it sits as
/// the first content section after the header prose.
const MATERIALS_ANCHOR: &str = "## Materials Summary";

/// Slice 9.5 — the h2-header anchor for cf-cast's generic Smooth-On
/// guidance (`design/cf-cast/src/procedure.rs:123` v1 + `:353` v2).
/// We inject the `## Slacker Recipe` section above this anchor so
/// it sits immediately after `## Materials Summary` — the recipe is
/// an enrichment of the materials table.
const GUIDANCE_ANCHOR: &str = "## Generic Smooth-On Guidance";

/// The h2 anchor for cf-cast's per-layer bench steps. v1 and v2 emit
/// `## Per-Layer Procedure`; the bonded path emits
/// `## Per-Layer Procedure (bonded, cast-in-place)`, so a prefix match
/// covers all three. Anchoring here rather than searching the whole
/// document is load-bearing: the `## Slacker Recipe` section this
/// module injects carries `### Layer {i} — ...` headings of its own,
/// and a document-wide search would hit those first.
const PER_LAYER_ANCHOR: &str = "## Per-Layer Procedure";

/// Slice 9.5 — per-layer Slacker recipe input. Carries everything
/// the procedure surface needs in one row: the layer's display name,
/// the cavity-fill volume + mass (lifted from cf-cast's `PourVolume`),
/// and the optional Slacker mass fraction (None if this layer doesn't
/// use Slacker — the section is then skipped for that row's
/// contribution, and emits no section at all if every row is None).
#[derive(Debug, Clone)]
pub struct SlackerLayerRecipe {
    /// Material display name (e.g., `"Ecoflex 00-30"`) — must match
    /// the entry in cf-cast's `## Materials Summary` table for the
    /// same layer index so the user can cross-reference rows.
    pub display_name: String,
    /// Cavity-fill mass for the BASE silicone alone, in kilograms
    /// (`cf_cast::PourVolume::pour_mass_kg` = `shell_volume_m3 *
    /// base_density`). Because the cured base+Slacker mix density is
    /// taken ≈ the base silicone's (Slacker publishes no density — see
    /// `inject_slacker_recipe_into_markdown`), this is also the
    /// COMBINED base+Slacker mix mass that fills the cavity; the base is
    /// scaled down from it, NOT topped up with Slacker.
    pub pour_mass_kg: f64,
    /// Cavity-fill VOLUME in cubic metres (`cf_cast::PourVolume::shell_volume_m3`)
    /// — the fixed target the mix must fill. Drives the base TDS specific
    /// gravity (`pour_mass_kg / shell_volume_m3`, = the material density)
    /// for the mL columns and the displayed cavity volume.
    pub shell_volume_m3: f64,
    /// Optional Slacker mass fraction (0.0–1.0) relative to the BASE
    /// (Part A + Part B) mass, matching the Slacker TB convention
    /// (`100B + 50 Slacker + 100A` ⇒ `50/200 = 0.25`). `None` means the
    /// layer does NOT use Slacker — its row is omitted from the table.
    pub slacker_fraction: Option<f64>,
}

/// Inject a `## Press-Fit Reservation` section into `procedure.md`
/// when `cavity_inset_m > 0`.
///
/// No-op for `cavity_inset_m == 0.0` (the inline-layers path), so
/// procedure markdown produced by cast.toml configs predating
/// slice 9.6 is bit-exact-preserved.
///
/// # Errors
///
/// - propagates filesystem read/write errors with file-path context.
/// - bails with a clear message if the `## Materials Summary` anchor
///   isn't present (means cf-cast's procedure shape changed — caller
///   should regenerate test fixtures or update the anchor).
pub fn inject_press_fit_section(path: &Path, cavity_inset_m: f64) -> Result<()> {
    if cavity_inset_m == 0.0 {
        return Ok(());
    }
    let original = std::fs::read_to_string(path).with_context(|| {
        format!(
            "read procedure.md at {} for press-fit injection",
            path.display()
        )
    })?;
    let patched = inject_press_fit_into_markdown(&original, cavity_inset_m).with_context(|| {
        format!(
            "inject press-fit section into procedure.md at {}",
            path.display()
        )
    })?;
    std::fs::write(path, patched)
        .with_context(|| format!("write patched procedure.md at {}", path.display()))?;
    Ok(())
}

/// Pure-string variant of [`inject_press_fit_section`] — used by the
/// unit test and the live path equally so a regression in the anchor
/// detection or section template surfaces in the cheap test layer
/// before the integration test.
pub(crate) fn inject_press_fit_into_markdown(
    original: &str,
    cavity_inset_m: f64,
) -> Result<String> {
    if !original.contains(MATERIALS_ANCHOR) {
        bail!(
            "procedure.md missing the `{}` anchor (cf-cast shape changed?)",
            MATERIALS_ANCHOR
        );
    }
    let inset_mm = cavity_inset_m * 1e3;
    let section = format!(
        "## Press-Fit Reservation\n\
         \n\
         The plug + every silicone layer is offset inward by **{inset_mm:.2} mm** \
         from the scan surface. The cured part's inner cavity is correspondingly \
         {inset_mm:.2} mm smaller than the scan, leaving a press-fit interference \
         for the real device to snap into. This is baked into the mold geometry \
         by `cf-cast-cli` from `cf-device-design`'s `cavity.inset_m`.\n\
         \n",
    );
    // Splice the new section in BEFORE `## Materials Summary` — string
    // `replace_n(1)` works because the anchor appears exactly once in
    // the v2 procedure markdown (verified by the `single anchor` unit
    // test below).
    let patched = original.replacen(MATERIALS_ANCHOR, &format!("{section}{MATERIALS_ANCHOR}"), 1);
    Ok(patched)
}

/// Slice 9.5 — inject a `## Slacker Recipe` section into
/// `procedure.md` when at least one layer uses Smooth-On Slacker
/// (`slacker_fraction > 0`).
///
/// No-op when every layer's `slacker_fraction` is `None` (the
/// inline-layers path, OR a design.toml with all-zero Slacker
/// fractions). The skip preserves bit-exact procedure markdown for
/// the no-Slacker case.
///
/// # Errors
///
/// - propagates filesystem read/write errors with file-path context.
/// - bails if the `## Generic Smooth-On Guidance` anchor isn't
///   present (cf-cast shape changed — the test pins anchor presence
///   on the v2 fixture).
pub fn inject_slacker_recipe_section(path: &Path, layers: &[SlackerLayerRecipe]) -> Result<()> {
    if !layers.iter().any(|l| l.slacker_fraction.is_some()) {
        return Ok(());
    }
    let original = std::fs::read_to_string(path).with_context(|| {
        format!(
            "read procedure.md at {} for slacker injection",
            path.display()
        )
    })?;
    let patched = inject_slacker_recipe_into_markdown(&original, layers).with_context(|| {
        format!(
            "inject slacker recipe section into procedure.md at {}",
            path.display()
        )
    })?;
    std::fs::write(path, patched)
        .with_context(|| format!("write patched procedure.md at {}", path.display()))?;
    Ok(())
}

/// Pure-string variant of [`inject_slacker_recipe_section`] — the
/// unit tests cover this so a markdown-shape regression surfaces
/// without hitting disk.
pub(crate) fn inject_slacker_recipe_into_markdown(
    original: &str,
    layers: &[SlackerLayerRecipe],
) -> Result<String> {
    if !original.contains(GUIDANCE_ANCHOR) {
        bail!(
            "procedure.md missing the `{}` anchor (cf-cast shape changed?)",
            GUIDANCE_ANCHOR
        );
    }

    // Build the recipe — one subsection per layer that opts in to Slacker.
    // Layers without Slacker (None) are skipped; if every entry is None the
    // file-path caller returns Ok early, so reaching here means ≥1 emits.
    //
    // VOLUME-CORRECT MIX (the fix to the earlier overfill): the cavity-fill
    // VOLUME is the fixed target. Slacker is mixed BY WEIGHT as a fraction of
    // the BASE (Part A + Part B, 1:1) mass — `slacker = sf · base` — so the
    // base+Slacker mixture TOGETHER fills the cavity; the base is scaled DOWN,
    // not topped up with Slacker on a full base pour. Smooth-On's Slacker TB
    // publishes no density, so the mix density is taken ≈ the base silicone's
    // (the documented "close enough" approximation already used in the
    // sim-design recipe path), under which the cavity-fill mix mass equals the
    // base's cavity-fill mass `pour_mass_kg`:
    //     base + slacker = pour_mass_g  ⇒  base = pour_mass_g / (1 + sf)
    // A flat +20% over-pour (sprue fill + cup cling + degas foam) is applied to
    // that CORRECTED total — never stacked on a base-full + Slacker overfill.
    const KG_TO_G: f64 = 1000.0;
    const M3_TO_CC: f64 = 1.0e6;
    const OVERAGE: f64 = 1.20; // +20% over-pour
    let mut out = String::new();
    // The per-layer bench steps are corrected in place on this copy as
    // the table is built; `out` is spliced in at the anchor at the end.
    let mut body = original.to_string();
    out.push_str("## Slacker Recipe\n\n");
    out.push_str(
        "**Smooth-On Slacker** softens the base silicone. It is mixed BY WEIGHT \
         (the Slacker TB requires a gram scale and publishes no specific gravity) \
         as a fraction of the **base** mass, where base = Part A + Part B (1:1 by \
         weight) — e.g. 25% Slacker is `100B + 50 Slacker, then 100A`. The base + \
         Slacker mixture TOGETHER fills the cavity: the base is scaled down so the \
         combined volume matches the cavity, NOT a full base pour with Slacker \
         added on top (which would overfill by the Slacker fraction). Cure is \
         anchored to the base silicone's TDS (Slacker only lengthens cure time).\n\n",
    );
    out.push_str(
        "Specific gravities — base silicone from the Smooth-On TDS (per layer \
         below; e.g. Ecoflex 00-30 = 1.07 g/cc). **Slacker has no published SG**, \
         so the mix density is taken ≈ the base silicone's; measure Slacker by \
         weight. Part A / Part B volumes use the base TDS SG; the Slacker volume \
         is not separately resolvable from a TDS value (weight only). The +20% \
         column is the over-pour to mix (sprue fill, cup cling, degas foam).\n\n",
    );
    for (idx, recipe) in layers.iter().enumerate() {
        let Some(sf) = recipe.slacker_fraction else {
            continue;
        };
        // Cavity-fill total mix mass (= base cavity-fill mass under mix≈base
        // density) and the base TDS specific gravity (= material density).
        let mix_fill_g = recipe.pour_mass_kg * KG_TO_G;
        let cavity_cc = recipe.shell_volume_m3 * M3_TO_CC;
        let sg = if recipe.shell_volume_m3 > 0.0 {
            recipe.pour_mass_kg / recipe.shell_volume_m3 / KG_TO_G // kg/m³ → g/cc
        } else {
            0.0
        };
        // Split: base = mix/(1+sf), each part = base/2, slacker = sf·base.
        let base_fill_g = mix_fill_g / (1.0 + sf);
        let part_fill_g = base_fill_g / 2.0;
        let slacker_fill_g = sf * base_fill_g;
        // Same locals, same layer: correct the bench step cf-cast wrote
        // without knowing this layer takes Slacker. Done here rather
        // than in a second pass so the step and the table row below can
        // never state different masses.
        body = correct_mix_step_for_layer(
            &body,
            idx,
            part_fill_g,
            base_fill_g,
            slacker_fill_g,
            sf * 100.0,
        )
        .with_context(|| format!("correct the Slacker mix step for layer {idx}"))?;
        // +20% over-pour on the corrected total (ratios preserved).
        let part_g = part_fill_g * OVERAGE;
        let slacker_g = slacker_fill_g * OVERAGE;
        let total_g = mix_fill_g * OVERAGE;
        // Volumes on the over-pour, base TDS SG (Slacker weight-only).
        let part_ml = if sg > 0.0 { part_g / sg } else { 0.0 };
        let total_ml = if sg > 0.0 { total_g / sg } else { 0.0 };

        let _ = std::fmt::Write::write_fmt(
            &mut out,
            format_args!(
                "### Layer {idx} — {name}, {pct:.0}% Slacker\n\n\
                 Cavity-fill volume ≈ {cavity_cc:.0} mL (fixed target) · base SG {sg:.2} g/cc.\n\n\
                 | Component | Cavity-fill (g) | +20% pour (g) | +20% pour (mL) |\n\
                 |-----------|----------------:|--------------:|---------------:|\n\
                 | Part A | {part_fill_g:.2} | {part_g:.2} | {part_ml:.1} |\n\
                 | Part B | {part_fill_g:.2} | {part_g:.2} | {part_ml:.1} |\n\
                 | Slacker ({pct:.0}% of base) | {slacker_fill_g:.2} | {slacker_g:.2} | weight only |\n\
                 | **Total mix** | **{mix_fill_g:.2}** | **{total_g:.2}** | **≈{total_ml:.0}** |\n\n",
                name = recipe.display_name,
                pct = sf * 100.0,
            ),
        );
    }

    let patched = body.replacen(GUIDANCE_ANCHOR, &format!("{out}{GUIDANCE_ANCHOR}"), 1);
    Ok(patched)
}

/// Rewrite one layer's mix step so it states the split the
/// `## Slacker Recipe` table specifies.
///
/// cf-cast renders that step from `pour_mass_kg` alone — it has no
/// Slacker concept at all (see this module's header), so for a Slacker
/// layer it halves the base+Slacker TOTAL into two parts of pure base:
/// `61.87 g Part A + 61.87 g Part B` for a 123.74 g mix that should be
/// `49.49 + 49.49 + 24.75`. Followed literally that pours a layer with
/// no Slacker in it; cross-referenced against the recipe table and
/// topped up instead, it overfills the cavity by the Slacker fraction —
/// the very error the recipe prose warns against. Both are silent, and
/// no keyword search finds either, because the mix step never says
/// "Slacker": the sheet's two halves describe one pour in different
/// words.
///
/// The masses come from the caller's loop — the SAME locals that build
/// the table row — so step and table cannot drift apart.
///
/// # Errors
///
/// Bails when the anchor, the layer heading, or a recognized mix-step
/// shape is missing. A silent no-op would restore precisely the defect
/// this exists to remove, so every miss is loud.
fn correct_mix_step_for_layer(
    body: &str,
    layer_idx: usize,
    part_fill_g: f64,
    base_fill_g: f64,
    slacker_fill_g: f64,
    pct: f64,
) -> Result<String> {
    let anchor = body.find(PER_LAYER_ANCHOR).with_context(|| {
        format!("procedure.md missing the `{PER_LAYER_ANCHOR}` anchor (cf-cast shape changed?)")
    })?;
    let heading = format!("### Layer {layer_idx} — ");
    let heading_at = body[anchor..]
        .find(&heading)
        .map(|rel| anchor + rel)
        .with_context(|| format!("no `{heading}` heading under `{PER_LAYER_ANCHOR}`"))?;
    // Bound the search to THIS layer's section, so a layer whose own
    // step is missing cannot silently pick up the next layer's.
    let after_heading = heading_at + heading.len();
    let section_end = body[after_heading..]
        .find("\n### ")
        .map_or(body.len(), |rel| after_heading + rel);
    let section = &body[heading_at..section_end];

    // Two shapes carry a mix step. The numeric split states a FALSE
    // base mass for a Slacker layer; the TDS fallback cf-cast emits for
    // a material with no known protocol states no split at all, so it
    // is incomplete rather than wrong and only needs the Slacker mass.
    const SPLIT_MARKER: &str = " g Part A + ";
    const TDS_MARKER: &str = "Mix Part A + Part B ";
    let (hit, is_split) = match (section.find(SPLIT_MARKER), section.find(TDS_MARKER)) {
        (Some(i), _) => (i, true),
        (None, Some(i)) => (i, false),
        (None, None) => bail!(
            "layer {layer_idx} takes Slacker but its `{PER_LAYER_ANCHOR}` section has no \
             recognized mix step (looked for `{SPLIT_MARKER}` and `{TDS_MARKER}`)"
        ),
    };
    let line_start = section[..hit].rfind('\n').map_or(0, |p| p + 1);
    let line_end = section[hit..]
        .find('\n')
        .map_or(section.len(), |rel| hit + rel);
    let line = &section[line_start..line_end];

    let corrected =
        if is_split {
            // Rebuild the two part masses, but reuse the material name, the
            // mix ratio and the trailing total VERBATIM — cf-cast owns
            // those and the Slacker split changes none of them.
            const MIX_LEAD: &str = ". Mix ";
            const PART_B: &str = " g Part B ";
            const RATIO_END: &str = " mix ratio)";
            let lead_end = line.find(MIX_LEAD).with_context(|| {
                format!("layer {layer_idx} mix step has no `{MIX_LEAD}`: {line}")
            })? + MIX_LEAD.len();
            let part_b_end = line
                .find(PART_B)
                .with_context(|| format!("layer {layer_idx} mix step has no `{PART_B}`: {line}"))?
                + PART_B.len();
            let ratio_end = line[part_b_end..].find(RATIO_END).with_context(|| {
                format!("layer {layer_idx} mix step has no `{RATIO_END}`: {line}")
            })? + part_b_end
                + RATIO_END.len();
            format!(
                "{}{part_fill_g:.2} g Part A + {part_fill_g:.2} g Part B {} plus \
             {slacker_fill_g:.2} g Slacker ({pct:.0}% of the {base_fill_g:.2} g base, by \
             weight; mix order per `## Slacker Recipe`){}",
                &line[..lead_end],
                &line[part_b_end..ratio_end],
                &line[ratio_end..],
            )
        } else {
            format!(
                "{line} Of that total, {slacker_fill_g:.2} g is Slacker ({pct:.0}% of the \
             {base_fill_g:.2} g base, by weight) — mix order per `## Slacker Recipe`."
            )
        };

    let abs_start = heading_at + line_start;
    let abs_end = heading_at + line_end;
    Ok(format!(
        "{}{corrected}{}",
        &body[..abs_start],
        &body[abs_end..]
    ))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;

    const FIXTURE_V2: &str = "\
# Cast Procedure\n\
\n\
Generated by `cf-cast` for a 1-layer cast (innermost-first).\n\
\n\
## Cast Geometry\n\
\n\
Some geometry prose.\n\
\n\
## Materials Summary\n\
\n\
| Layer | Material | Pour Mass |\n\
|------:|----------|----------:|\n\
| 0 | Ecoflex 00-30 | 142.0 g |\n\
\n\
## Generic Smooth-On Guidance\n\
\n\
Some guidance.\n\
\n\
## Per-Layer Procedure\n\
\n\
### Layer 0 — Ecoflex 00-30 (innermost)\n\
\n\
1. Print STLs.\n\
4. Mix 71.00 g Part A + 71.00 g Part B Ecoflex 00-30 (1A:1B mix ratio). Total 142.00 g.\n\
5. Vacuum-degas the mix.\n\
\n\
### Layer 1 — Dragon Skin 10A (outermost)\n\
\n\
1. Print STLs.\n\
4. Mix 40.00 g Part A + 40.00 g Part B Dragon Skin 10A (1A:1B mix ratio). Total 80.00 g.\n\
5. Vacuum-degas the mix.\n";

    #[test]
    fn inject_press_fit_inserts_section_above_materials() {
        let patched = inject_press_fit_into_markdown(FIXTURE_V2, 0.002).unwrap();
        let press_fit_idx = patched
            .find("## Press-Fit Reservation")
            .expect("Press-Fit section present");
        let materials_idx = patched
            .find("## Materials Summary")
            .expect("Materials section preserved");
        assert!(
            press_fit_idx < materials_idx,
            "Press-Fit must precede Materials; got press_fit={press_fit_idx}, materials={materials_idx}",
        );
        assert!(
            patched.contains("**2.00 mm**"),
            "inset value (2 mm) must appear in mm with 2-dp formatting; got:\n{patched}",
        );
        // Original sections all preserved verbatim.
        assert!(patched.contains("## Cast Geometry"));
        assert!(patched.contains("## Generic Smooth-On Guidance"));
        assert!(patched.contains("Ecoflex 00-30"));
    }

    #[test]
    fn inject_press_fit_is_noop_at_zero_inset() {
        // The file-path path is the no-op gate; the string variant
        // pins behavior for non-zero inset values. inset = 0 doesn't
        // reach the string injection in normal flow, but if a caller
        // forces it, we still emit a section (with 0.00 mm — exact
        // semantics for an explicit zero is debatable but consistent
        // with the floor: callers who want no-op should use the
        // file-path version or check inset before calling).
        let patched = inject_press_fit_into_markdown(FIXTURE_V2, 0.0).unwrap();
        assert!(patched.contains("**0.00 mm**"));
    }

    #[test]
    fn inject_press_fit_missing_anchor_errors() {
        let no_materials = "# Cast Procedure\n\n## Just A Header\n";
        let err = inject_press_fit_into_markdown(no_materials, 0.002).unwrap_err();
        assert!(
            err.to_string().contains("Materials Summary"),
            "unexpected error message: {err}",
        );
    }

    #[test]
    fn materials_anchor_appears_once_in_fixture() {
        // Anchor uniqueness is what makes `replacen(1)` safe — if
        // cf-cast ever emits the header twice, this trips here.
        assert_eq!(FIXTURE_V2.matches(MATERIALS_ANCHOR).count(), 1);
    }

    // Cavity-fill mass at the Ecoflex-00-30 TDS density (1070 kg/m³, SG 1.07),
    // so the derived base SG in the table comes out 1.07 g/cc.
    fn slacker_layer(name: &str, mass_kg: f64, sf: Option<f64>) -> SlackerLayerRecipe {
        SlackerLayerRecipe {
            display_name: name.to_string(),
            pour_mass_kg: mass_kg,
            shell_volume_m3: mass_kg / 1070.0,
            slacker_fraction: sf,
        }
    }

    #[test]
    fn inject_slacker_recipe_inserts_table_above_guidance() {
        let layers = vec![
            slacker_layer("Ecoflex 00-30", 0.142, Some(0.10)),
            slacker_layer("Dragon Skin 10A", 0.080, Some(0.05)),
        ];
        let patched = inject_slacker_recipe_into_markdown(FIXTURE_V2, &layers).unwrap();
        let slacker_idx = patched
            .find("## Slacker Recipe")
            .expect("Slacker Recipe section present");
        let guidance_idx = patched
            .find("## Generic Smooth-On Guidance")
            .expect("Generic Smooth-On Guidance section preserved");
        assert!(
            slacker_idx < guidance_idx,
            "Slacker Recipe must precede Guidance; got slacker={slacker_idx}, guidance={guidance_idx}",
        );
        // Materials still upstream of Slacker.
        let materials_idx = patched.find(MATERIALS_ANCHOR).unwrap();
        assert!(
            materials_idx < slacker_idx,
            "Materials must remain above Slacker; got materials={materials_idx}, slacker={slacker_idx}",
        );
        // Layer 0: 142.00 g total mix fills the cavity (NOT base-full + Slacker
        // on top). base = 142/1.10 = 129.09 → A=B=64.55 g, Slacker = 12.91 g;
        // base+Slacker = 142.00 (no overfill). +20% over-pour: total 170.40 g,
        // Slacker 15.49 g. Base SG 1.07; Slacker is weight-only.
        assert!(
            patched.contains("### Layer 0 — Ecoflex 00-30, 10% Slacker")
                && patched.contains("142.00") // cavity-fill total mix
                && patched.contains("12.91") // Slacker (cavity-fill)
                && patched.contains("170.40") // +20% total
                && patched.contains("15.49") // Slacker (+20%)
                && patched.contains("weight only")
                && patched.contains("1.07"), // base TDS SG
            "layer-0 recipe not formatted as expected; got:\n{patched}",
        );
        // base+Slacker == cavity-fill mass (no overfill): 64.55+64.55+12.91 = 142.00.
        assert!(
            !patched.contains("156.20") // 142 + 14.20 (the old base-full+Slacker overfill)
                && !patched.contains("14.20 g"),
            "must not reproduce the base-full + Slacker-on-top overfill; got:\n{patched}",
        );
        // Layer 1: 80.00 g total mix, +20% = 96.00 g.
        assert!(
            patched.contains("### Layer 1 — Dragon Skin 10A, 5% Slacker")
                && patched.contains("80.00")
                && patched.contains("96.00"),
            "layer-1 recipe not formatted as expected; got:\n{patched}",
        );
    }

    #[test]
    fn inject_slacker_recipe_omits_no_slacker_rows() {
        // Layer 1 has slacker_fraction = None → omitted from table.
        let layers = vec![
            slacker_layer("Ecoflex 00-30", 0.142, Some(0.15)),
            slacker_layer("Dragon Skin 10A", 0.080, None),
        ];
        let patched = inject_slacker_recipe_into_markdown(FIXTURE_V2, &layers).unwrap();
        assert!(patched.contains("## Slacker Recipe"));
        // Layer 0's recipe present: 15% Slacker, base = 142/1.15 = 123.48 →
        // Slacker (cavity-fill) = 18.52 g, base+Slacker = 142.00 (no overfill).
        assert!(
            patched.contains("### Layer 0 — Ecoflex 00-30, 15% Slacker")
                && patched.contains("18.52")
                && patched.contains("142.00"),
            "layer-0 (15% Slacker) recipe not formatted as expected; got:\n{patched}",
        );
        // Not the old overfill (142 * 0.15 = 21.30 g Slacker on a full base pour).
        assert!(!patched.contains("21.30 g"));
        // Layer 1's material name does not appear in the Slacker
        // section (it WILL appear in cf-cast's existing
        // `## Materials Summary` since the fixture has it too —
        // restrict the search to the Slacker section).
        let slacker_start = patched.find("## Slacker Recipe").unwrap();
        let guidance_start = patched.find("## Generic Smooth-On Guidance").unwrap();
        let slacker_section = &patched[slacker_start..guidance_start];
        assert!(
            !slacker_section.contains("Dragon Skin 10A"),
            "non-slacker layer 1 must not appear in Slacker Recipe section; got section:\n{slacker_section}",
        );
    }

    #[test]
    fn inject_slacker_recipe_missing_anchor_errors() {
        let layers = vec![slacker_layer("Ecoflex 00-30", 0.142, Some(0.10))];
        let no_guidance = "# Cast Procedure\n\n## Materials Summary\n\n| header |\n";
        let err = inject_slacker_recipe_into_markdown(no_guidance, &layers).unwrap_err();
        assert!(
            err.to_string().contains("Generic Smooth-On Guidance"),
            "unexpected error message: {err}",
        );
    }

    #[test]
    fn guidance_anchor_appears_once_in_fixture() {
        assert_eq!(FIXTURE_V2.matches(GUIDANCE_ANCHOR).count(), 1);
    }

    /// The defect this module exists to close: cf-cast writes the mix
    /// step from the pour mass alone, so a Slacker layer's step used to
    /// halve the base+Slacker TOTAL into two parts of pure base. The
    /// second assertion is the negative control — it re-states the
    /// uncorrected split verbatim and requires it to be gone.
    #[test]
    fn slacker_layer_mix_step_states_the_slacker_split() {
        let layers = vec![
            slacker_layer("Ecoflex 00-30", 0.142, Some(0.10)),
            slacker_layer("Dragon Skin 10A", 0.080, None),
        ];
        let patched = inject_slacker_recipe_into_markdown(FIXTURE_V2, &layers).unwrap();
        let steps = &patched[patched.find(PER_LAYER_ANCHOR).unwrap()..];

        // base = 142.00 / 1.10 = 129.09 -> A = B = 64.55, Slacker = 12.91.
        assert!(
            steps.contains(
                "4. Mix 64.55 g Part A + 64.55 g Part B Ecoflex 00-30 (1A:1B mix ratio) \
                 plus 12.91 g Slacker (10% of the 129.09 g base, by weight; mix order \
                 per `## Slacker Recipe`). Total 142.00 g."
            ),
            "layer-0 mix step not corrected; got:\n{steps}",
        );
        assert!(
            !steps.contains("71.00 g Part A"),
            "the uncorrected 1A:1B split of the TOTAL mix is still present; got:\n{steps}",
        );
    }

    #[test]
    fn non_slacker_layer_mix_step_is_untouched() {
        let layers = vec![
            slacker_layer("Ecoflex 00-30", 0.142, Some(0.10)),
            slacker_layer("Dragon Skin 10A", 0.080, None),
        ];
        let patched = inject_slacker_recipe_into_markdown(FIXTURE_V2, &layers).unwrap();
        assert!(
            patched.contains(
                "4. Mix 40.00 g Part A + 40.00 g Part B Dragon Skin 10A (1A:1B mix ratio). \
                 Total 80.00 g."
            ),
            "layer 1 takes no Slacker; its step must survive verbatim. Got:\n{patched}",
        );
        assert!(
            !patched.contains("Dragon Skin 10A (1A:1B mix ratio) plus"),
            "layer 1 must not gain a Slacker clause; got:\n{patched}",
        );
    }

    #[test]
    fn slacker_layer_without_a_mix_step_errors() {
        // A silent no-op here would re-open the defect, so a per-layer
        // section carrying no recognized mix step must be loud. This
        // also pins the section bound: layer 0's search must NOT fall
        // through to layer 1's still-present step.
        let no_step = FIXTURE_V2.replace(
            "4. Mix 71.00 g Part A + 71.00 g Part B Ecoflex 00-30 (1A:1B mix ratio). \
             Total 142.00 g.\n",
            "",
        );
        assert!(
            !no_step.contains("71.00 g Part A"),
            "fixture surgery failed — the step is still there, so the test proves nothing",
        );
        let layers = vec![slacker_layer("Ecoflex 00-30", 0.142, Some(0.10))];
        let err = inject_slacker_recipe_into_markdown(&no_step, &layers).unwrap_err();
        let msg = format!("{err:#}");
        assert!(
            msg.contains("no recognized mix step"),
            "unexpected error message: {msg}",
        );
    }

    #[test]
    fn tds_fallback_mix_step_gains_the_slacker_mass() {
        // cf-cast emits this shape when the material has no known
        // Smooth-On protocol. It states no A/B split, so it is
        // incomplete rather than wrong — it needs the Slacker mass
        // appended, not the numbers rebuilt.
        let tds = FIXTURE_V2.replace(
            "4. Mix 71.00 g Part A + 71.00 g Part B Ecoflex 00-30 (1A:1B mix ratio). \
             Total 142.00 g.",
            "4. Mix Part A + Part B Ecoflex 00-30 per the Smooth-On TDS (total pour \
             mass: 142.00 g, split per the data-sheet mix ratio).",
        );
        let layers = vec![slacker_layer("Ecoflex 00-30", 0.142, Some(0.10))];
        let patched = inject_slacker_recipe_into_markdown(&tds, &layers).unwrap();
        assert!(
            patched.contains(
                "Of that total, 12.91 g is Slacker (10% of the 129.09 g base, by weight) \
                 — mix order per `## Slacker Recipe`."
            ),
            "TDS-fallback step did not gain the Slacker mass; got:\n{patched}",
        );
    }

    #[test]
    fn per_layer_anchor_appears_once_in_fixture() {
        assert_eq!(FIXTURE_V2.matches(PER_LAYER_ANCHOR).count(), 1);
    }
}
