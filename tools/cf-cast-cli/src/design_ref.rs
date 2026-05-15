//! Slice 9 — read the `.design.toml` produced by cf-device-design.
//!
//! This is the **read-only mirror** of `cf-device-design::design_toml`'s
//! schema. Both sides of the workspace deserialize the same TOML
//! format, gated by a shared `schema_version` constant. Format-drift
//! between the two is caught at load time by
//! [`validate_design_ref`]: a load with `schema_version >
//! DESIGN_TOML_SCHEMA_VERSION_READ` is rejected with a precise error.
//!
//! Why duplicate instead of share via a new workspace crate? At slice
//! 9 there are two consumers (cf-device-design + cf-cast-cli) and the
//! schema is small (≈50 LOC of structs + validation). A future third
//! consumer would justify extracting `cf-device-design-format` as a
//! lib crate; until then, the parallel-source-of-truth cost is lower
//! than the workspace-edit + dep-graph cost. See
//! `docs/INSERTION_SIM_STATE.md`'s slice-9 update for the rationale.
//!
//! ## Read-only
//!
//! cf-cast-cli never *writes* a design TOML — it consumes one
//! produced by cf-device-design. So the structs here are
//! `Deserialize`-only and there is no `save_design_*` / `build_*`
//! function. Authoritative emit-side lives in
//! `tools/cf-device-design/src/design_toml.rs`.

use std::path::Path;

use anyhow::{Result, anyhow, ensure};
use serde::Deserialize;

/// Highest design.toml `schema_version` this binary accepts. Must
/// match `cf-device-design::design_toml::DESIGN_TOML_SCHEMA_VERSION`;
/// a mismatch surfaces at parse time as a forward-incompat error.
pub const DESIGN_TOML_SCHEMA_VERSION_READ: u32 = 1;

/// Layer-count cap mirrored from cf-device-design's `LAYER_COUNT_MAX`.
/// Validation rejects design.toml files that exceed it.
pub const LAYER_COUNT_MAX_READ: usize = 6;

// ── on-disk serde mirrors ───────────────────────────────────────────

/// Top-level `.design.toml` mirror — see
/// `cf-device-design::design_toml::DesignToml` for the emit-side.
#[derive(Debug, Clone, Deserialize)]
pub struct DesignRef {
    pub device_design: DesignMetaBlock,
    pub scan_ref: ScanRefBlock,
    pub cavity: CavityBlock,
    pub layers: Vec<LayerBlock>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DesignMetaBlock {
    pub tool_version: String,
    pub generated_at: String,
    pub schema_version: u32,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ScanRefBlock {
    /// Informational — cf-cast-cli's `[scan].cleaned_stl` is
    /// authoritative for the loaded scan; this records what
    /// cf-device-design was designing against.
    pub cleaned_stl: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CavityBlock {
    /// Press-fit interference reservation (meters). Today this is
    /// informational only at the cast-cli level — the plug = scan
    /// (Option A) so the mold geometry doesn't currently apply the
    /// inset. The procedure markdown records it.
    pub inset_m: f64,
    /// Visibility flag from cf-device-design's viewport. Not
    /// consumed at the cast-cli level.
    pub visible: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LayerBlock {
    pub thickness_m: f64,
    pub material_anchor_key: String,
    /// Slacker mass fraction. Today informational only at the
    /// cast-cli level (cf-cast doesn't model Slacker in mold
    /// geometry). A future enhancement can lift this into the
    /// procedure markdown's per-layer recipe block.
    pub slacker_fraction: f64,
    pub visible: bool,
}

// ── load + validate ─────────────────────────────────────────────────

/// Load + validate a `.design.toml`. Cf-cast-cli wraps any error
/// with the file path via `Context`.
///
/// # Errors
///
/// - file unreadable / TOML malformed,
/// - `schema_version > DESIGN_TOML_SCHEMA_VERSION_READ`,
/// - structural / range invariants violated (see
///   [`validate_design_ref`]).
pub fn load_design_ref(path: &Path) -> Result<DesignRef> {
    let text =
        std::fs::read_to_string(path).map_err(|e| anyhow!("read {} ({e:#})", path.display()))?;
    let design: DesignRef =
        toml::from_str(&text).map_err(|e| anyhow!("parse TOML at {} ({e:#})", path.display()))?;
    validate_design_ref(&design)?;
    Ok(design)
}

/// Structural validation — same gates as cf-device-design's
/// `validate_design_toml`.
///
/// # Errors
///
/// - `schema_version > DESIGN_TOML_SCHEMA_VERSION_READ` (forward-incompat).
/// - `layers.is_empty()` or `layers.len() > LAYER_COUNT_MAX_READ`.
/// - Any layer's `thickness_m` is non-finite or `<= 0`.
/// - Any layer's `slacker_fraction` is non-finite or `< 0`.
/// - `cavity.inset_m` is non-finite or `< 0`.
///
/// Anchor-key catalog membership is **NOT** validated here — cf-cast-cli
/// has its own anchor catalog via `density_for_anchor`, with a
/// `density_kg_m3` override path for unrecognized anchors. The
/// design's key flows through and gets that same treatment at
/// `CastConfig::validate` time.
pub fn validate_design_ref(design: &DesignRef) -> Result<()> {
    if design.device_design.schema_version > DESIGN_TOML_SCHEMA_VERSION_READ {
        return Err(anyhow!(
            "design.toml schema_version {} is newer than this cf-cast-cli supports \
             ({}). Update cf-cast-cli.",
            design.device_design.schema_version,
            DESIGN_TOML_SCHEMA_VERSION_READ,
        ));
    }
    ensure!(
        !design.layers.is_empty(),
        "design.toml has no layers — at least one required"
    );
    ensure!(
        design.layers.len() <= LAYER_COUNT_MAX_READ,
        "design.toml has {} layers — exceeds the {LAYER_COUNT_MAX_READ}-layer cap",
        design.layers.len(),
    );
    for (i, l) in design.layers.iter().enumerate() {
        ensure!(
            l.thickness_m.is_finite() && l.thickness_m > 0.0,
            "design.toml layer {i} thickness_m = {} must be finite + > 0",
            l.thickness_m,
        );
        ensure!(
            l.slacker_fraction.is_finite() && l.slacker_fraction >= 0.0,
            "design.toml layer {i} slacker_fraction = {} must be finite + >= 0",
            l.slacker_fraction,
        );
    }
    ensure!(
        design.cavity.inset_m.is_finite() && design.cavity.inset_m >= 0.0,
        "design.toml cavity.inset_m = {} must be finite + >= 0",
        design.cavity.inset_m,
    );
    Ok(())
}

// ── tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    /// Hand-written TOML that mirrors what cf-device-design writes,
    /// so a real round-trip is exercised at the schema level.
    const SAMPLE: &str = r#"
[device_design]
tool_version = "1.0.0"
generated_at = "2026-05-15T22:34:00Z"
schema_version = 1

[scan_ref]
cleaned_stl = "sock_over_capsule.cleaned.stl"

[cavity]
inset_m = 0.003
visible = true

[[layers]]
thickness_m = 0.005
material_anchor_key = "ECOFLEX_00_30"
slacker_fraction = 0.0
visible = true

[[layers]]
thickness_m = 0.004
material_anchor_key = "DRAGON_SKIN_10A"
slacker_fraction = 0.25
visible = true
"#;

    #[test]
    fn parses_a_cf_device_design_emission() {
        let design: DesignRef = toml::from_str(SAMPLE).unwrap();
        validate_design_ref(&design).unwrap();
        assert_eq!(design.device_design.schema_version, 1);
        assert_eq!(design.scan_ref.cleaned_stl, "sock_over_capsule.cleaned.stl");
        assert!((design.cavity.inset_m - 0.003).abs() < 1e-12);
        assert_eq!(design.layers.len(), 2);
        assert_eq!(design.layers[0].material_anchor_key, "ECOFLEX_00_30");
        assert_eq!(design.layers[1].material_anchor_key, "DRAGON_SKIN_10A");
        assert!((design.layers[1].slacker_fraction - 0.25).abs() < 1e-12);
    }

    #[test]
    fn future_schema_version_rejected() {
        let bumped = SAMPLE.replace("schema_version = 1", "schema_version = 99");
        let design: DesignRef = toml::from_str(&bumped).unwrap();
        let err = validate_design_ref(&design).unwrap_err();
        assert!(
            err.to_string()
                .contains("newer than this cf-cast-cli supports")
        );
    }

    #[test]
    fn empty_layers_rejected() {
        // `layers = []` is at the top so it sets the root-level
        // field; if it followed a `[section]` header it would land
        // inside that section instead. The `[[layers]]` array of
        // tables form is omitted entirely.
        let text = r#"
layers = []

[device_design]
tool_version = "1.0.0"
generated_at = "2026-05-15T22:34:00Z"
schema_version = 1

[scan_ref]
cleaned_stl = "x.cleaned.stl"

[cavity]
inset_m = 0.003
visible = true
"#;
        let design: DesignRef = toml::from_str(text).unwrap();
        let err = validate_design_ref(&design).unwrap_err();
        assert!(err.to_string().contains("no layers"));
    }

    #[test]
    fn nonpositive_thickness_rejected() {
        let bad = SAMPLE.replace("thickness_m = 0.005", "thickness_m = -0.001");
        let design: DesignRef = toml::from_str(&bad).unwrap();
        let err = validate_design_ref(&design).unwrap_err();
        assert!(err.to_string().contains("thickness_m"));
    }

    #[test]
    fn negative_slacker_rejected() {
        let bad = SAMPLE.replace("slacker_fraction = 0.0", "slacker_fraction = -0.1");
        let design: DesignRef = toml::from_str(&bad).unwrap();
        let err = validate_design_ref(&design).unwrap_err();
        assert!(err.to_string().contains("slacker_fraction"));
    }

    #[test]
    fn negative_cavity_inset_rejected() {
        let bad = SAMPLE.replace("inset_m = 0.003", "inset_m = -0.001");
        let design: DesignRef = toml::from_str(&bad).unwrap();
        let err = validate_design_ref(&design).unwrap_err();
        assert!(err.to_string().contains("cavity.inset_m"));
    }

    #[test]
    fn layer_count_overflow_rejected() {
        // 7 layers > LAYER_COUNT_MAX_READ = 6.
        let mut text = String::from(
            r#"
[device_design]
tool_version = "1.0.0"
generated_at = "2026-05-15T22:34:00Z"
schema_version = 1

[scan_ref]
cleaned_stl = "x.cleaned.stl"

[cavity]
inset_m = 0.003
visible = true
"#,
        );
        for _ in 0..7 {
            text.push_str(
                r#"
[[layers]]
thickness_m = 0.001
material_anchor_key = "ECOFLEX_00_30"
slacker_fraction = 0.0
visible = true
"#,
            );
        }
        let design: DesignRef = toml::from_str(&text).unwrap();
        let err = validate_design_ref(&design).unwrap_err();
        assert!(err.to_string().contains("exceeds the"));
    }
}
