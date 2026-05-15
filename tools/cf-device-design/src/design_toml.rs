//! `design_toml` — slice 8 Save/Open for `<scan>.design.toml`.
//!
//! Persists the design panel's state (`CavityState` + `LayersState`)
//! to a TOML file alongside the cleaned STL, so a session can be
//! resumed from a later run via the `--design <path>` flag (or
//! auto-default to `<cleaned-stl-stem>.design.toml`).
//!
//! Slice 7's insertion-sim is *not* persisted — the sim runs fresh
//! each session. Only the design intent (cavity inset + layer stack +
//! per-layer Slacker fraction) round-trips.
//!
//! ## Schema
//!
//! ```toml
//! [device_design]
//! tool_version = "1.0.0"
//! generated_at = "2026-05-15T22:34:00Z"
//! schema_version = 1
//!
//! [scan_ref]
//! cleaned_stl = "sock_over_capsule.cleaned.stl"  # relative or absolute
//!
//! [cavity]
//! inset_m = 0.003
//! visible = true
//!
//! [[layers]]
//! thickness_m = 0.005
//! material_anchor_key = "ECOFLEX_00_30"
//! slacker_fraction = 0.0
//! visible = true
//!
//! [[layers]]
//! thickness_m = 0.005
//! material_anchor_key = "DRAGON_SKIN_10A"
//! slacker_fraction = 0.25
//! visible = true
//! ```
//!
//! `schema_version` is a forward-compat marker: a load with a future
//! version (`>` the one this binary writes) is rejected with a clear
//! error rather than misinterpreted; an older version (`<`) is
//! accepted today (slice-8 schema is the first version, so this only
//! becomes interesting if 8.x+ ever migrates).

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow};
use serde::{Deserialize, Serialize};

use crate::{CavityState, LAYER_COUNT_MAX, LAYER_MATERIALS, LayerSpec, LayersState};

/// The schema version this binary writes + accepts. Bump when the
/// TOML structure changes; loads of a higher version are rejected
/// with a precise error.
pub const DESIGN_TOML_SCHEMA_VERSION: u32 = 1;

// ── on-disk serde structs ───────────────────────────────────────────

/// Top-level `.design.toml`. Mirrors the schema documented at module
/// top.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DesignToml {
    /// Tool + schema metadata.
    pub device_design: DesignMetaBlock,
    /// Reference to the cleaned scan this design targets.
    pub scan_ref: ScanRefBlock,
    /// Cavity state — inset distance + visibility.
    pub cavity: CavityBlock,
    /// Ordered (innermost-first) layer stack.
    pub layers: Vec<LayerBlock>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DesignMetaBlock {
    /// `env!("CARGO_PKG_VERSION")` at write time. Provenance only —
    /// loads don't gate on this.
    pub tool_version: String,
    /// ISO 8601 UTC timestamp at write time.
    pub generated_at: String,
    /// Schema version — see [`DESIGN_TOML_SCHEMA_VERSION`].
    pub schema_version: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanRefBlock {
    /// Path to the cleaned STL. Stored verbatim; `--cleaned_stl` on
    /// reopen wins (the design TOML's `cleaned_stl` is informational
    /// — the CLI's path is authoritative). Records what the design
    /// was built against for an audit trail.
    pub cleaned_stl: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CavityBlock {
    pub inset_m: f64,
    pub visible: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerBlock {
    pub thickness_m: f64,
    pub material_anchor_key: String,
    pub slacker_fraction: f64,
    pub visible: bool,
}

// ── path resolution ─────────────────────────────────────────────────

/// Resolve the design-TOML path: explicit `--design <PATH>` wins;
/// else fall back to `<cleaned-stl-stem-minus-.cleaned>.design.toml`
/// next to the cleaned STL (parallels `resolve_prep_toml_path` in
/// `main.rs`). Returns `None` only for the extremely-rare bare-
/// filename-with-no-parent case.
pub fn resolve_design_toml_path(cleaned_stl: &Path, explicit: Option<&Path>) -> Option<PathBuf> {
    if let Some(p) = explicit {
        return Some(p.to_path_buf());
    }
    let parent = cleaned_stl.parent()?;
    let stem = cleaned_stl.file_stem()?.to_str()?;
    let base_stem = stem.strip_suffix(".cleaned").unwrap_or(stem);
    Some(parent.join(format!("{base_stem}.design.toml")))
}

// ── serialize from app state ────────────────────────────────────────

/// Build a [`DesignToml`] from the current app state.
///
/// `cavity.inset_m` and `layer.thickness_m` are rounded to **1 µm
/// precision** before serialization. The egui sliders operate on
/// continuous `f64` values whose IEEE-754 binary representation
/// often picks up ~1-ULP noise — dragging to "3.1 mm" stores
/// `0.0031000000000000003 m`, and `toml::to_string_pretty` emits all
/// 17 significant digits. Rounding to micrometers (`1e-6 m =
/// 0.001 mm`) is well below any meaningful slider resolution
/// (sliders show 1 decimal mm) and lets the TOML emitter pick the
/// shortest-round-trip form, so the on-disk file reads "0.0031" not
/// "0.0031000000000000003". Round-trip is bit-stable since the load
/// path reads the rounded value verbatim. Visual-pass finding 5 on
/// the slice-7-9 PR review.
pub fn build_design_toml(
    cleaned_stl: &Path,
    cavity: &CavityState,
    layers: &LayersState,
) -> DesignToml {
    DesignToml {
        device_design: DesignMetaBlock {
            tool_version: env!("CARGO_PKG_VERSION").to_string(),
            generated_at: iso8601_utc_now(),
            schema_version: DESIGN_TOML_SCHEMA_VERSION,
        },
        scan_ref: ScanRefBlock {
            cleaned_stl: cleaned_stl.display().to_string(),
        },
        cavity: CavityBlock {
            inset_m: round_to_micrometers(cavity.inset_m),
            visible: cavity.visible,
        },
        layers: layers
            .layers
            .iter()
            .map(|l| LayerBlock {
                thickness_m: round_to_micrometers(l.thickness_m),
                material_anchor_key: l.material_anchor_key.to_string(),
                // `slacker_fraction` is discrete (`0.0`, `0.25`, `0.5`,
                // `0.75`, `1.00` per the TB curve) and the values are
                // all exactly representable in f64; no rounding
                // needed here.
                slacker_fraction: l.slacker_fraction,
                visible: l.visible,
            })
            .collect(),
    }
}

/// Round an `f64` value in meters to the nearest micrometer
/// (`1e-6 m`). Cleans up IEEE-754 noise from slider-driven f64
/// values; well below any meaningful slider resolution (sliders are
/// 0.1 mm = 1e-4 m). Non-finite values pass through unchanged so
/// `validate_design_toml`'s `is_finite()` gate sees the original
/// (and the user gets the original bug message rather than a
/// rounded one).
fn round_to_micrometers(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    (x * 1_000_000.0).round() / 1_000_000.0
}

/// Save a [`DesignToml`] to disk. Atomic via `<path>.tmp` + rename so
/// a partial write never overwrites a previously-good file.
///
/// # Errors
///
/// - TOML serialization fails (shouldn't, structurally; `f64` always
///   serializes).
/// - The `.tmp` file can't be created / written.
/// - The atomic rename fails (filesystem-level; cleanup attempted).
pub fn save_design_toml(design: &DesignToml, path: &Path) -> Result<()> {
    let body = toml::to_string_pretty(design).context("serialize DesignToml")?;
    let tmp = path.with_extension("toml.tmp");
    std::fs::write(&tmp, &body).with_context(|| format!("write tmp file {}", tmp.display()))?;
    if let Err(e) = std::fs::rename(&tmp, path) {
        // Best-effort cleanup so we don't leave the .tmp around if
        // the rename failed.
        let _ = std::fs::remove_file(&tmp);
        return Err(anyhow::Error::new(e).context(format!(
            "rename {} → {}",
            tmp.display(),
            path.display()
        )));
    }
    Ok(())
}

// ── load from disk + apply to state ─────────────────────────────────

/// Load + validate a `.design.toml`. Rejects unknown schema versions
/// + unknown anchor keys + per-layer-count out of range.
///
/// # Errors
///
/// - File can't be read.
/// - TOML parse fails.
/// - `schema_version > DESIGN_TOML_SCHEMA_VERSION`.
/// - `layers.is_empty()` or `layers.len() > LAYER_COUNT_MAX`.
/// - Any layer's `material_anchor_key` isn't in `LAYER_MATERIALS`.
pub fn load_design_toml(path: &Path) -> Result<DesignToml> {
    let text = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    let design: DesignToml =
        toml::from_str(&text).with_context(|| format!("parse TOML at {}", path.display()))?;
    validate_design_toml(&design)?;
    Ok(design)
}

/// Structural validation of a parsed [`DesignToml`]. Pulled out so
/// the tests can run it on hand-built fixtures.
pub fn validate_design_toml(design: &DesignToml) -> Result<()> {
    if design.device_design.schema_version > DESIGN_TOML_SCHEMA_VERSION {
        return Err(anyhow!(
            "design.toml schema_version {} is newer than this binary supports ({}). \
             Update cf-device-design.",
            design.device_design.schema_version,
            DESIGN_TOML_SCHEMA_VERSION,
        ));
    }
    if design.layers.is_empty() {
        return Err(anyhow!(
            "design.toml has no layers — at least one layer required"
        ));
    }
    if design.layers.len() > LAYER_COUNT_MAX {
        return Err(anyhow!(
            "design.toml has {} layers — exceeds the {LAYER_COUNT_MAX}-layer cap",
            design.layers.len(),
        ));
    }
    for (i, l) in design.layers.iter().enumerate() {
        if !LAYER_MATERIALS
            .iter()
            .any(|(k, _, _)| *k == l.material_anchor_key)
        {
            return Err(anyhow!(
                "design.toml layer {i} names unknown anchor key {:?} — \
                 not in the cf-device-design catalog",
                l.material_anchor_key,
            ));
        }
        if !l.thickness_m.is_finite() || l.thickness_m <= 0.0 {
            return Err(anyhow!(
                "design.toml layer {i} has invalid thickness_m = {}",
                l.thickness_m,
            ));
        }
        if !l.slacker_fraction.is_finite() || l.slacker_fraction < 0.0 {
            return Err(anyhow!(
                "design.toml layer {i} has invalid slacker_fraction = {}",
                l.slacker_fraction,
            ));
        }
    }
    if !design.cavity.inset_m.is_finite() || design.cavity.inset_m < 0.0 {
        return Err(anyhow!(
            "design.toml cavity.inset_m = {} is invalid (must be finite, ≥ 0)",
            design.cavity.inset_m,
        ));
    }
    Ok(())
}

/// Apply a validated [`DesignToml`] to the live `(CavityState,
/// LayersState)` resources. The catalog-key lookup recovers the
/// `&'static str` reference that `LayerSpec` wants (anchor keys are
/// `'static` per the slice-5 design).
pub fn apply_design_toml(
    design: &DesignToml,
    cavity: &mut CavityState,
    layers: &mut LayersState,
) -> Result<()> {
    cavity.inset_m = design.cavity.inset_m;
    cavity.visible = design.cavity.visible;
    let mut new_layers: Vec<LayerSpec> = Vec::with_capacity(design.layers.len());
    for (i, l) in design.layers.iter().enumerate() {
        // Recover the `&'static str` anchor key by matching the
        // owned String against the catalog.
        let anchor_static: &'static str = LAYER_MATERIALS
            .iter()
            .find(|(k, _, _)| *k == l.material_anchor_key)
            .map(|(k, _, _)| *k)
            .ok_or_else(|| {
                anyhow!(
                    "design.toml layer {i} anchor {:?} not in catalog (validate_design_toml \
                     should have caught this)",
                    l.material_anchor_key,
                )
            })?;
        new_layers.push(LayerSpec {
            thickness_m: l.thickness_m,
            material_anchor_key: anchor_static,
            slacker_fraction: l.slacker_fraction,
            visible: l.visible,
        });
    }
    layers.layers = new_layers;
    Ok(())
}

// ── timestamp helper ────────────────────────────────────────────────

/// ISO 8601 UTC string for the current wall clock — `"YYYY-MM-DDTHH:MM:SSZ"`.
/// Lifted from cf-scan-prep's `chrono_like_timestamp` to avoid a
/// shared utility crate for one function. Same Howard-Hinnant
/// civil-from-days algorithm.
fn iso8601_utc_now() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let days = secs / 86_400;
    let secs_in_day = secs % 86_400;
    let hours = secs_in_day / 3600;
    let minutes = (secs_in_day % 3600) / 60;
    let seconds = secs_in_day % 60;
    let (year, month, day) = unix_days_to_ymd(days as i64);
    format!("{year:04}-{month:02}-{day:02}T{hours:02}:{minutes:02}:{seconds:02}Z")
}

/// Howard-Hinnant civil-from-days. Identical to cf-scan-prep's copy.
#[allow(clippy::cast_possible_wrap, clippy::cast_sign_loss)]
fn unix_days_to_ymd(z: i64) -> (i64, u32, u32) {
    let z_shifted = z + 719_468;
    let era = if z_shifted >= 0 {
        z_shifted / 146_097
    } else {
        (z_shifted - 146_096) / 146_097
    };
    let doe = (z_shifted - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = if m <= 2 { y + 1 } else { y };
    (year, m, d)
}

// ── tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use super::*;

    /// Path inference matches the cf-scan-prep convention: strip
    /// `.cleaned` from the STL stem to recover the source-scan stem,
    /// then append `.design.toml` next to it.
    #[test]
    fn resolve_design_toml_path_strips_cleaned_suffix() {
        let stl = PathBuf::from("/tmp/sock_over_capsule.cleaned.stl");
        let resolved = resolve_design_toml_path(&stl, None).unwrap();
        assert_eq!(
            resolved,
            PathBuf::from("/tmp/sock_over_capsule.design.toml")
        );
    }

    /// Plain stems (no `.cleaned` suffix) are preserved verbatim.
    #[test]
    fn resolve_design_toml_path_handles_stem_without_cleaned_suffix() {
        let stl = PathBuf::from("/tmp/sample.stl");
        let resolved = resolve_design_toml_path(&stl, None).unwrap();
        assert_eq!(resolved, PathBuf::from("/tmp/sample.design.toml"));
    }

    /// Explicit `--design <PATH>` wins over the auto-default.
    #[test]
    fn resolve_design_toml_path_honors_explicit_override() {
        let stl = PathBuf::from("/tmp/scan.cleaned.stl");
        let explicit = PathBuf::from("/elsewhere/named.design.toml");
        let resolved = resolve_design_toml_path(&stl, Some(&explicit)).unwrap();
        assert_eq!(resolved, explicit);
    }

    fn cavity_state() -> CavityState {
        CavityState {
            inset_m: 0.003,
            visible: true,
        }
    }

    fn layers_state_two_layers() -> LayersState {
        LayersState {
            layers: vec![
                LayerSpec {
                    thickness_m: 0.005,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                },
                LayerSpec {
                    thickness_m: 0.004,
                    material_anchor_key: "DRAGON_SKIN_10A",
                    slacker_fraction: 0.25,
                    visible: false,
                },
            ],
        }
    }

    /// Build → save → load → apply round-trip preserves every field.
    #[test]
    fn round_trip_preserves_state() {
        let cavity = cavity_state();
        let layers = layers_state_two_layers();
        let design = build_design_toml(std::path::Path::new("scan.cleaned.stl"), &cavity, &layers);

        // Serialize → parse → apply (in-memory; no FS).
        let body = toml::to_string_pretty(&design).unwrap();
        let parsed: DesignToml = toml::from_str(&body).unwrap();
        validate_design_toml(&parsed).unwrap();

        let mut cavity_loaded = CavityState {
            inset_m: 0.0,
            visible: false,
        };
        let mut layers_loaded = LayersState { layers: vec![] };
        apply_design_toml(&parsed, &mut cavity_loaded, &mut layers_loaded).unwrap();

        assert_eq!(cavity_loaded.inset_m, cavity.inset_m);
        assert_eq!(cavity_loaded.visible, cavity.visible);
        assert_eq!(layers_loaded.layers.len(), layers.layers.len());
        for (a, b) in layers_loaded.layers.iter().zip(layers.layers.iter()) {
            assert_eq!(a.thickness_m, b.thickness_m);
            assert_eq!(a.material_anchor_key, b.material_anchor_key);
            assert_eq!(a.slacker_fraction, b.slacker_fraction);
            assert_eq!(a.visible, b.visible);
        }
    }

    /// Schema version newer than this binary → loud error.
    #[test]
    fn future_schema_version_rejected() {
        let mut design = build_design_toml(
            std::path::Path::new("scan.cleaned.stl"),
            &cavity_state(),
            &layers_state_two_layers(),
        );
        design.device_design.schema_version = DESIGN_TOML_SCHEMA_VERSION + 1;
        let err = validate_design_toml(&design).unwrap_err();
        assert!(err.to_string().contains("newer than this binary supports"));
    }

    /// Empty layer list is rejected.
    #[test]
    fn empty_layers_rejected() {
        let mut design = build_design_toml(
            std::path::Path::new("scan.cleaned.stl"),
            &cavity_state(),
            &layers_state_two_layers(),
        );
        design.layers.clear();
        let err = validate_design_toml(&design).unwrap_err();
        assert!(err.to_string().contains("no layers"));
    }

    /// Layer count beyond `LAYER_COUNT_MAX` is rejected.
    #[test]
    fn layer_count_overflow_rejected() {
        let mut design = build_design_toml(
            std::path::Path::new("scan.cleaned.stl"),
            &cavity_state(),
            &layers_state_two_layers(),
        );
        // Pad with default layers until we exceed the cap.
        while design.layers.len() <= LAYER_COUNT_MAX {
            design.layers.push(LayerBlock {
                thickness_m: 0.001,
                material_anchor_key: "ECOFLEX_00_30".to_string(),
                slacker_fraction: 0.0,
                visible: true,
            });
        }
        let err = validate_design_toml(&design).unwrap_err();
        assert!(err.to_string().contains("exceeds the"));
    }

    /// Unknown anchor key rejected with the bad key named.
    #[test]
    fn unknown_anchor_key_rejected() {
        let mut design = build_design_toml(
            std::path::Path::new("scan.cleaned.stl"),
            &cavity_state(),
            &layers_state_two_layers(),
        );
        design.layers[0].material_anchor_key = "NOT_A_SILICONE".to_string();
        let err = validate_design_toml(&design).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("NOT_A_SILICONE"), "got: {msg}");
        assert!(msg.contains("not in the cf-device-design catalog"));
    }

    /// Negative thickness rejected; NaN rejected.
    #[test]
    fn invalid_thickness_rejected() {
        let mut design = build_design_toml(
            std::path::Path::new("scan.cleaned.stl"),
            &cavity_state(),
            &layers_state_two_layers(),
        );
        design.layers[0].thickness_m = -0.001;
        assert!(validate_design_toml(&design).is_err());
        design.layers[0].thickness_m = f64::NAN;
        assert!(validate_design_toml(&design).is_err());
    }

    /// Negative cavity inset rejected; NaN rejected.
    #[test]
    fn invalid_cavity_inset_rejected() {
        let mut design = build_design_toml(
            std::path::Path::new("scan.cleaned.stl"),
            &cavity_state(),
            &layers_state_two_layers(),
        );
        design.cavity.inset_m = -0.001;
        assert!(validate_design_toml(&design).is_err());
        design.cavity.inset_m = f64::NAN;
        assert!(validate_design_toml(&design).is_err());
    }

    /// Atomic save → load round-trip on a real `tempfile`-style path.
    #[test]
    fn save_and_load_round_trip_on_disk() {
        let dir =
            std::env::temp_dir().join(format!("cf-device-design-7-8-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("scan.design.toml");

        let cavity = cavity_state();
        let layers = layers_state_two_layers();
        let design = build_design_toml(std::path::Path::new("scan.cleaned.stl"), &cavity, &layers);
        save_design_toml(&design, &path).unwrap();

        let loaded = load_design_toml(&path).unwrap();
        let mut c = CavityState {
            inset_m: 0.0,
            visible: false,
        };
        let mut l = LayersState { layers: vec![] };
        apply_design_toml(&loaded, &mut c, &mut l).unwrap();
        assert_eq!(c.inset_m, cavity.inset_m);
        assert_eq!(l.layers.len(), 2);

        // Cleanup.
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir(&dir);
    }

    /// `round_to_micrometers` cleans up IEEE-754 noise without
    /// shifting the meaning. `0.0031000000000000003` (the f64
    /// value the cavity-slider stores when dragged to "3.1 mm")
    /// rounds to the IEEE-754 canonical form of `0.0031`, which
    /// `f64::to_string` then emits as `"0.0031"` (the shortest
    /// round-trip).
    #[test]
    fn round_to_micrometers_strips_ieee754_noise() {
        // The noisy form the slider produces.
        let noisy = 0.0031000000000000003_f64;
        let clean = round_to_micrometers(noisy);
        // The clean form should serialize without 17-digit noise.
        // We can't compare bit-equal because f64(0.0031) is itself
        // not the exact decimal — but the toml-emitter (via Rust's
        // `f64::to_string`) picks the shortest round-trip, so a
        // `format!("{clean}")` should not contain the noise tail.
        let s = format!("{clean}");
        assert!(
            !s.contains("0000000000000003"),
            "rounded value should not carry the IEEE-754 noise tail: {s}"
        );
        // And the rounded value should be within 1 nm of the original
        // (negligible compared to the 0.1 mm slider grid).
        assert!(
            (clean - noisy).abs() < 1e-9,
            "rounding shifted by more than 1 nm: noisy={noisy} clean={clean}"
        );
    }

    /// Non-finite values pass through `round_to_micrometers`
    /// unchanged so the downstream `validate_design_toml` gate sees
    /// the original NaN / inf and produces the precise diagnostic.
    #[test]
    fn round_to_micrometers_pass_through_nonfinite() {
        assert!(round_to_micrometers(f64::NAN).is_nan());
        assert_eq!(round_to_micrometers(f64::INFINITY), f64::INFINITY);
        assert_eq!(round_to_micrometers(f64::NEG_INFINITY), f64::NEG_INFINITY);
    }

    /// End-to-end: build a `DesignToml` from a state whose
    /// `inset_m` carries IEEE-754 noise, serialize to TOML, and
    /// confirm the on-disk string carries the clean form.
    #[test]
    fn build_design_toml_emits_clean_decimals() {
        let cavity = CavityState {
            inset_m: 0.0031000000000000003,
            visible: true,
        };
        let layers = LayersState {
            layers: vec![LayerSpec {
                thickness_m: 0.010000000000000002,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.25,
                visible: true,
            }],
        };
        let design = build_design_toml(std::path::Path::new("scan.cleaned.stl"), &cavity, &layers);
        let body = toml::to_string_pretty(&design).unwrap();
        // The cleaned values should serialize without the long
        // IEEE-754 noise tail.
        assert!(
            body.contains("inset_m = 0.0031"),
            "expected clean 'inset_m = 0.0031' in:\n{body}"
        );
        assert!(
            body.contains("thickness_m = 0.01"),
            "expected clean 'thickness_m = 0.01' in:\n{body}"
        );
        assert!(
            !body.contains("0.0031000000000000003"),
            "the long noise form should not appear: {body}"
        );
    }

    /// `iso8601_utc_now` returns something that *looks* right —
    /// 20 chars, ends in `Z`, contains a `T`.
    #[test]
    fn iso_timestamp_well_formed() {
        let ts = iso8601_utc_now();
        assert_eq!(ts.len(), 20, "got {ts:?}");
        assert!(ts.ends_with('Z'));
        assert!(ts.contains('T'));
    }
}
