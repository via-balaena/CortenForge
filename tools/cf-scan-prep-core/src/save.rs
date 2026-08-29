//! Timestamps and the atomic cleaned-scan write path.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use crate::{
    DetectedCapLoop, bake_vertex_with_pivot, emit_centroid_fan_cap, project_loop_to_plane_2d,
};
use anyhow::{Context, Result};
use mesh_io::save_stl;
use mesh_types::{Bounded, IndexedMesh, Point3};
use nalgebra::{UnitQuaternion, Vector3};
use std::path::Path;

/// Minimal RFC 3339-ish timestamp without pulling in `chrono`. Uses
/// the OS clock + `std::time::SystemTime` for a UTC-shaped string
/// good enough for provenance. If the clock returns an error
/// (extremely unusual), falls back to a placeholder.
pub fn chrono_like_timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    // Plain seconds-since-epoch (the user's timezone is irrelevant
    // for provenance — they can convert if needed). Format as ISO
    // 8601 UTC by computing a calendar date from the unix timestamp.
    iso8601_utc_from_unix_seconds(secs)
}

/// Convert a unix timestamp (seconds since 1970-01-01 UTC) into an
/// ISO 8601 UTC string like `2026-05-12T22:34:00Z`. Inline because
/// the only alternative is pulling in `chrono` for one function.
pub fn iso8601_utc_from_unix_seconds(unix_secs: u64) -> String {
    // Days since epoch.
    let days = unix_secs / 86_400;
    let secs_in_day = unix_secs % 86_400;
    let hours = secs_in_day / 3600;
    let minutes = (secs_in_day % 3600) / 60;
    let seconds = secs_in_day % 60;

    // Walk forward from 1970-01-01 day-by-day. Slow for far-future
    // dates but trivial for "now". Use a calendar table.
    let (year, month, day) = unix_days_to_ymd(days as i64);
    format!("{year:04}-{month:02}-{day:02}T{hours:02}:{minutes:02}:{seconds:02}Z")
}

/// Convert "days since 1970-01-01" to a (year, month, day) tuple.
/// Algorithm: Howard Hinnant's civil-from-days; well-known + branch-
/// less. ~30 LOC inline, avoids the `chrono` dep.
pub fn unix_days_to_ymd(z: i64) -> (i64, u32, u32) {
    // Shift so the "year 0" anchor is March 1 of year 0 (so leap
    // days fall at year boundaries cleanly).
    let z_shifted = z + 719_468;
    let era = if z_shifted >= 0 {
        z_shifted / 146_097
    } else {
        (z_shifted - 146_096) / 146_097
    };
    let doe = (z_shifted - era * 146_097) as u64; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    let year = if m <= 2 { y + 1 } else { y };
    (year, m, d)
}

/// Atomic two-file write: writes `cleaned_stl_path` + `prep_toml_path`
/// to `.tmp` siblings first, then renames both to final names. If
/// either step fails, BOTH `.tmp` files are cleaned up so the user
/// doesn't end up with a half-written set. Spec §Architectural
/// decisions §Save atomicity.
pub fn atomic_write_save(
    cleaned_mesh: &IndexedMesh,
    cleaned_stl_path: &Path,
    prep_toml_path: &Path,
    prep_toml_content: &str,
) -> Result<()> {
    let stl_tmp = cleaned_stl_path.with_extension("stl.tmp");
    let toml_tmp = prep_toml_path.with_extension("toml.tmp");

    // STL write (binary; cleaned scans are large — text STL would
    // 10x the file size with no benefit).
    save_stl(cleaned_mesh, &stl_tmp, true)
        .with_context(|| format!("writing {}", stl_tmp.display()))?;
    // TOML write.
    if let Err(e) = std::fs::write(&toml_tmp, prep_toml_content) {
        // Roll back STL tmp; surface the TOML error.
        let _ = std::fs::remove_file(&stl_tmp);
        return Err(anyhow::Error::new(e).context(format!("writing {}", toml_tmp.display())));
    }
    // Atomic renames.
    if let Err(e) = std::fs::rename(&stl_tmp, cleaned_stl_path) {
        let _ = std::fs::remove_file(&stl_tmp);
        let _ = std::fs::remove_file(&toml_tmp);
        return Err(anyhow::Error::new(e).context(format!("renaming {}", stl_tmp.display())));
    }
    if let Err(e) = std::fs::rename(&toml_tmp, prep_toml_path) {
        // STL already landed; remove it to keep atomicity contract
        // ("neither final file lands if either write fails").
        let _ = std::fs::remove_file(cleaned_stl_path);
        let _ = std::fs::remove_file(&toml_tmp);
        return Err(anyhow::Error::new(e).context(format!("renaming {}", toml_tmp.display())));
    }
    Ok(())
}

/// Build the cleaned IndexedMesh from the working scan + Reorient +
/// Recenter + included cap loops.
///
/// Pipeline:
/// 1. Clone `scan` into `out`.
/// 2. Bake rotation + translation into vertex positions (in place).
///    After this step `out.vertices` are in world frame.
/// 3. For each included cap loop, ear-clip its 2D projection and
///    append triangles (using existing loop vertex indices — no new
///    vertices added). Cap faces inherit the world-frame coordinates
///    from step 2.
///
/// CSP.4c — Clip-floor step removed (the feature itself retired
/// alongside `ClipState`). Centerline trim handles the workshop
/// "shave the noisy end" use case the clip used to cover.
///
/// **Cap normal orientation**: the spec mandates that the cap's
/// outward normal point away from the mesh interior. We orient the
/// triangulation's vertex winding to match the loop's stored
/// `plane_normal` (which `build_detected_cap_loop` already flipped
/// to face outward).
pub fn build_cleaned_mesh(
    scan: &IndexedMesh,
    rotation: UnitQuaternion<f64>,
    translation_m: Vector3<f64>,
    cap_loops: &[DetectedCapLoop],
) -> IndexedMesh {
    let translation = translation_m;
    // Pivot the bake rotation around the raw scan AABB centroid so
    // rotation rotates the mesh in place (matches the viewport's
    // centroid-pivot Transform composition). Empty mesh → centroid
    // at origin (matches `IndexedMesh::aabb()` returning
    // `Aabb::empty()`); the loop below is a no-op anyway.
    let centroid = scan.aabb().center();

    let mut out = scan.clone();

    // Step 2: bake transforms. Compute centroid-pivoted rotation +
    // recenter translation for each vertex from the ORIGINAL
    // (pre-bake) coordinates so cap triangulation can still reference
    // the original positions for plane fit etc. We then mutate
    // `out.vertices` in place.
    for v in out.vertices.iter_mut() {
        *v = bake_vertex_with_pivot(v, rotation, &centroid, translation);
    }

    // Step 3: triangulate + append included caps. Loop vertex
    // indices were captured at scan time + reference positions in
    // `scan.0.vertices` — which after our in-place transform are
    // now in world frame. So both the original-loop-positions math
    // (for the 2D projection) AND the resulting face indices stay
    // valid.
    for cap_loop in cap_loops {
        if !cap_loop.include {
            continue;
        }
        // Loop's stored `plane_centroid` and `plane_normal` are in
        // pre-bake physics-local frame. Bake centroid through the
        // same pivot transform as the vertices; rotate the plane
        // normal (a direction, no pivot or translation).
        let centroid_world =
            bake_vertex_with_pivot(&cap_loop.plane_centroid, rotation, &centroid, translation);
        let normal_world = rotation.transform_vector(&cap_loop.plane_normal);

        // CSP.3a — project each loop vertex onto the fit plane
        // BEFORE the ear-clip. The boundary loop's points have
        // sub-mm wobble for typical R² ≈ 0.9 fixtures; without this
        // projection the resulting cap faces lie OFF the plane
        // (because ear-clip's 2D projection is into the plane, but
        // the final 3D faces draw from the unprojected vertex
        // positions). Snapping the rim onto the plane keeps the cap
        // planar at the cost of moving the scan-body rim ring by
        // the wobble magnitude — sub-mm, well below the 2 mm SDF
        // cell + below the meshopt target_error threshold. The
        // alternative (append separate cap vertices) leaves a
        // sub-mm seam gap that breaks watertightness for mesh_sdf.
        // All loop indices are validated below; we project only the
        // ones that exist in the current `out.vertices`.
        for &idx in &cap_loop.vertex_indices {
            if let Some(p) = out.vertices.get(idx as usize).copied() {
                let signed = (p.coords - centroid_world.coords).dot(&normal_world);
                let projected = p.coords - normal_world * signed;
                out.vertices[idx as usize] = Point3::from(projected);
            }
        }

        let loop_points_world: Vec<Point3<f64>> = cap_loop
            .vertex_indices
            .iter()
            .filter_map(|&idx| out.vertices.get(idx as usize).copied())
            .collect();
        if loop_points_world.len() != cap_loop.vertex_indices.len() {
            // Some indices invalid (mesh changed after scan); skip
            // this loop rather than emit malformed faces.
            continue;
        }

        let verts_2d = project_loop_to_plane_2d(&loop_points_world, centroid_world, normal_world);

        // Centroid-fan replacement for the ear-clip path (S1.1
        // 2026-05-26) — produces a non-overlapping fan from a fresh
        // centroid vertex to each perimeter edge. Manifold by
        // construction for star-shaped projected polygons; eliminates
        // the duplicate-face + non-manifold-edge artifacts the ear-
        // clip's fan-fallback emitted on self-intersecting
        // projections. Same helper as `auto_cap_open_boundaries`.
        emit_centroid_fan_cap(
            &mut out,
            &cap_loop.vertex_indices,
            &loop_points_world,
            &verts_2d,
        );
    }

    out
}
