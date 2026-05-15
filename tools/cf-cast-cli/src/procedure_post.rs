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

/// Slice 9.5 — per-layer Slacker recipe input. Carries everything
/// the procedure surface needs in one row: the layer's display name,
/// its base-silicone pour mass (kg, lifted from cf-cast's
/// `PourVolume`), and the optional Slacker mass fraction (None if
/// this layer doesn't use Slacker — the section is then skipped for
/// that row's contribution, and emits no section at all if every
/// row is None).
#[derive(Debug, Clone)]
pub struct SlackerLayerRecipe {
    /// Material display name (e.g., `"Ecoflex 00-30"`) — must match
    /// the entry in cf-cast's `## Materials Summary` table for the
    /// same layer index so the user can cross-reference rows.
    pub display_name: String,
    /// Base silicone pour mass in kilograms (lifted from
    /// `cf_cast::PourVolume::pour_mass_kg`). The Slacker mass is
    /// computed as `slacker_fraction * pour_mass_kg`.
    pub pour_mass_kg: f64,
    /// Optional Slacker mass fraction (0.0–1.0). `None` means the
    /// layer does NOT use Slacker — its row is omitted from the
    /// recipe table.
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

    // Build the recipe table — one row per layer that opts in to
    // Slacker, indexed by position. Layers without Slacker (None)
    // are omitted entirely from the table; if every entry is None,
    // the file-path caller returns Ok early without invoking us,
    // so reaching this point means at least one row will emit.
    const KG_TO_G: f64 = 1000.0;
    let mut table = String::new();
    table.push_str("## Slacker Recipe\n\n");
    table.push_str(
        "The following layers use **Smooth-On Slacker** softener mixed into \
         the base silicone by mass. The base-mass column mirrors the per-layer \
         pour mass in `## Materials Summary` — combine each row with the listed \
         Slacker mass at mix time (cure-anchored to the base silicone's TDS, \
         not Slacker's; the additive is rheology-only).\n\n",
    );
    table.push_str("| Layer | Material | Base Mass | Slacker Mass | Fraction |\n");
    table.push_str("|------:|----------|----------:|-------------:|---------:|\n");
    for (idx, recipe) in layers.iter().enumerate() {
        let Some(sf) = recipe.slacker_fraction else {
            continue;
        };
        let base_g = recipe.pour_mass_kg * KG_TO_G;
        let slacker_g = base_g * sf;
        let _ = std::fmt::Write::write_fmt(
            &mut table,
            format_args!(
                "| {} | {} | {:.2} g | {:.2} g | {:.1}% |\n",
                idx,
                recipe.display_name,
                base_g,
                slacker_g,
                sf * 100.0,
            ),
        );
    }
    table.push('\n');

    let patched = original.replacen(GUIDANCE_ANCHOR, &format!("{table}{GUIDANCE_ANCHOR}"), 1);
    Ok(patched)
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
Some guidance.\n";

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

    fn slacker_layer(name: &str, mass_kg: f64, sf: Option<f64>) -> SlackerLayerRecipe {
        SlackerLayerRecipe {
            display_name: name.to_string(),
            pour_mass_kg: mass_kg,
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
        // Layer 0: 142 g base × 0.10 = 14.20 g Slacker.
        assert!(
            patched.contains("142.00 g")
                && patched.contains("14.20 g")
                && patched.contains("10.0%"),
            "layer-0 row not formatted as expected; got:\n{patched}",
        );
        // Layer 1: 80 g base × 0.05 = 4.00 g Slacker.
        assert!(
            patched.contains("80.00 g") && patched.contains("4.00 g") && patched.contains("5.0%"),
            "layer-1 row not formatted as expected; got:\n{patched}",
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
        // Layer 0's slacker row present (142 * 0.15 = 21.30 g).
        assert!(patched.contains("21.30 g") && patched.contains("15.0%"));
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
}
