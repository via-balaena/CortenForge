//! S0 empirical spike for the cf-cast Canal Interior recon
//! (`docs/CF_CAST_CANAL_INTERIOR_RECON.md` §13).
//!
//! ONE `#[ignore]`-gated probe — manually invoked with
//! `cargo test --release -p cf-cast --test s0_canal_probe
//!  -- --ignored --nocapture`. NO PRODUCTION COMMIT — this file is a
//! throwaway diagnostic scaffold (mirrors `s0_scan_mesh_direct_probe.rs`).
//! It builds a probe-local `CanalProbeSdf` (NOT the production
//! `CanalPlugSdf` — that lands in S2) and answers the three questions
//! the recon front-loads before any infrastructure is built:
//!
//! - **Q-resolution** — does the ~1.5 mm frenulum texture survive the
//!   mesher? Sweep uniform marching cubes at cell sizes
//!   {0.003 (prod), 0.001, 0.0005} m + one adaptive (DC) pass; report
//!   wall-clock, face/vertex count, self-intersection count, and the
//!   recovered texture amplitude (residual of frenulum-side vertices
//!   vs the smooth base radius). Decides the production mesher for S3.
//! - **Q-meshes-clean** — does the D-section + texture SDF mesh without
//!   self-intersection artifacts? Validates the hand-written SDF pattern
//!   before `RadiusLut` + config get built in S1/S2.
//! - **Q-pull-out (the showstopper)** — emit `canal_probe.stl` to
//!   `~/scans/canal_s0/` with a DELIBERATE mid-canal undercut (a chamber
//!   wider than its neck, ratio ~1.42). The workshop user prints it,
//!   pours scrap Ecoflex/Dragon Skin around it, and tries to pull the
//!   rigid plug out. This is the one constraint no code can answer; the
//!   whole risk-averse cadence hinges on this physical result coming back
//!   before S1 starts.

#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used,
    clippy::explicit_iter_loop,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::manual_assert,
    clippy::uninlined_format_args,
    clippy::default_trait_access,
    clippy::map_unwrap_or,
    clippy::doc_markdown,
    clippy::assigning_clones,
    clippy::missing_const_for_fn,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines,
    dead_code
)]

use std::path::PathBuf;
use std::time::Instant;

use cf_design::{Aabb, Sdf, Solid};
use mesh_io::save_stl;
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_repair::intersect::{IntersectionParams, detect_self_intersections};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

/// Cells of padding on each side of the AABB — matches `mesher.rs`'s
/// `GRID_PADDING_CELLS` so the inline MC loop behaves like production.
const GRID_PADDING_CELLS: usize = 2;
const METERS_TO_MM: f64 = 1000.0;

/// Budgeted canal length (m) — `L` in the recon zone model.
const PROBE_LENGTH_M: f64 = 0.12;
/// Base insert radius (m) — stands in for the scan girth budget `r_p`.
const PROBE_BASE_RADIUS_M: f64 = 0.018;
/// Texture amplitude (m) — the recon's starting `A ≈ 1.5 mm`.
const PROBE_TEXTURE_AMP_M: f64 = 0.0015;
/// Texture pitch (m) — annular rib spacing.
const PROBE_TEXTURE_PITCH_M: f64 = 0.008;
/// Frenulum-side radius factor (tight) for the D-section blend.
const FRENULUM_FACTOR: f64 = 0.50;
/// Dorsal-side radius factor (loose) for the D-section blend.
const DORSAL_FACTOR: f64 = 0.80;

// ────────────────────────────────────────────────────────────────────
// Probe-local parametric canal SDF
// ────────────────────────────────────────────────────────────────────

/// Probe-local parametric interior-canal plug SDF. Straight z-axis
/// (centerline = +Z), NO scan input — so this probe has zero external
/// dependencies and is trivially runnable. Its *negative* is the
/// designed cavity. Negative inside / positive outside, per the `Sdf`
/// contract.
///
/// Exercises the three mechanisms the production `CanalPlugSdf` needs:
/// 1. **Axial zones** — piecewise-linear radius profile `R(z)` over
///    `(t, radius_fraction)` control stations (entry ring → clearance
///    chamber → taper, or a neck/chamber/neck undercut).
/// 2. **D-section asymmetry** — radius scaled between `frenulum_factor`
///    (tight) and `dorsal_factor` (loose) by ½(1 + cos θ), where θ is
///    the azimuth from `frenulum_dir`.
/// 3. **Additive texture** — `A·sin(2π z / pitch)` gated by
///    `w(θ) = max(0, cos θ)` so ribs live on the frenulum side and fade
///    smooth dorsally.
#[derive(Clone)]
struct CanalProbeSdf {
    length_m: f64,
    base_radius_m: f64,
    /// `(t ∈ [0,1], radius fraction of base_radius_m)` stations, sorted
    /// ascending by `t`. Interpolated linearly; clamped at the ends.
    stations: Vec<(f64, f64)>,
    /// Frenulum direction (unit vector in the XY plane); the asymmetry
    /// axis. `θ = 0` points along this.
    frenulum_dir: Vector3<f64>,
    frenulum_factor: f64,
    dorsal_factor: f64,
    texture_amp_m: f64,
    texture_pitch_m: f64,
    /// Apply the additive frenulum-gated texture term.
    textured: bool,
    /// Apply the D-section angular compression.
    asymmetric: bool,
}

impl CanalProbeSdf {
    /// Linearly interpolate the radius-fraction profile at normalized
    /// axial position `t ∈ [0, 1]`. Clamped to the end stations.
    fn radius_frac(&self, t: f64) -> f64 {
        let st = &self.stations;
        if t <= st[0].0 {
            return st[0].1;
        }
        let last = st[st.len() - 1];
        if t >= last.0 {
            return last.1;
        }
        for w in st.windows(2) {
            let (t0, f0) = w[0];
            let (t1, f1) = w[1];
            if t >= t0 && t <= t1 {
                let a = if (t1 - t0).abs() < 1.0e-12 {
                    0.0
                } else {
                    (t - t0) / (t1 - t0)
                };
                return (f1 - f0).mul_add(a, f0);
            }
        }
        last.1
    }

    /// Smooth base radius at `(z, θ)` WITHOUT the additive texture term —
    /// the zone profile times the D-section factor. Used both inside
    /// `eval` and by the texture-recovery measurement to detrend.
    fn base_radius_at(&self, z: f64, cos_theta: f64) -> f64 {
        let t = (z / self.length_m).clamp(0.0, 1.0);
        let base = self.base_radius_m * self.radius_frac(t);
        if self.asymmetric {
            // R(z,θ) = R(z) · lerp(dorsal, frenulum, ½(1+cosθ)).
            // cosθ=+1 (frenulum side) → frenulum_factor (tight).
            // cosθ=-1 (dorsal side)   → dorsal_factor (loose).
            let s = 0.5 * (1.0 + cos_theta);
            base * (self.frenulum_factor - self.dorsal_factor).mul_add(s, self.dorsal_factor)
        } else {
            base
        }
    }

    /// Largest radius the surface reaches (for AABB sizing): max zone
    /// fraction × base + texture amplitude + margin. D-section only
    /// shrinks and texture only adds on the frenulum side, so this
    /// bounds the surface.
    fn max_radius_m(&self) -> f64 {
        let max_frac = self.stations.iter().map(|s| s.1).fold(0.0_f64, f64::max);
        self.base_radius_m.mul_add(max_frac, self.texture_amp_m) + 0.002
    }

    fn bounds(&self) -> Aabb {
        let r = self.max_radius_m();
        Aabb::new(
            Point3::new(-r, -r, -0.005),
            Point3::new(r, r, self.length_m + 0.005),
        )
    }
}

impl Sdf for CanalProbeSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let z = p.z;
        let r_xy = p.x.hypot(p.y);
        let cos_theta = if r_xy > 1.0e-9 {
            p.x.mul_add(self.frenulum_dir.x, p.y * self.frenulum_dir.y) / r_xy
        } else {
            0.0
        };

        let r_base = self.base_radius_at(z, cos_theta);

        let tex = if self.textured {
            let w = cos_theta.max(0.0);
            self.texture_amp_m * (2.0 * std::f64::consts::PI * z / self.texture_pitch_m).sin() * w
        } else {
            0.0
        };

        let r_total = r_base + tex;

        // Capped cylinder-ish solid: radial wall + flat end planes at
        // z=0 (mouth/base) and z=L (terminal). Intersection of
        // halfspaces = max() of the signed distances. Negative inside.
        let d_radial = r_xy - r_total;
        let d_low = -z;
        let d_high = z - self.length_m;
        d_radial.max(d_low).max(d_high)
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        // Unused: `Solid::from_sdf` bridges via `FieldNode::UserFn`,
        // which takes the gradient by finite differences. Same rationale
        // as `FlangeSdf`/`GasketChannelSdf`. Return the asymmetry axis.
        self.frenulum_dir
    }
}

// ────────────────────────────────────────────────────────────────────
// Meshing helpers (inline replication of mesher.rs — `mesher` is private)
// ────────────────────────────────────────────────────────────────────

/// Sample `solid`'s SDF onto a `ScalarGrid` at `cell_size_m`, run
/// marching cubes, and return the mesh in METERS (no mm scale — scale
/// only when saving STL). Replicates the ~20-line loop in
/// `cf-cast::mesher::solid_to_mm_mesh` so the probe doesn't widen the
/// production surface.
fn mesh_uniform(solid: &Solid, cell_size_m: f64) -> IndexedMesh {
    let bounds = solid.bounds().expect("solid has finite bounds");
    let mut grid = ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, GRID_PADDING_CELLS);
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, solid.evaluate(&p));
            }
        }
    }
    marching_cubes(&grid, &MarchingCubesConfig::default())
}

/// Multiply every vertex by `factor` in place (m → mm at the export
/// boundary).
fn scale_in_place(mesh: &mut IndexedMesh, factor: f64) {
    for v in &mut mesh.vertices {
        v.x *= factor;
        v.y *= factor;
        v.z *= factor;
    }
}

fn out_dir() -> PathBuf {
    let home = std::env::var("HOME").expect("HOME set");
    let dir = PathBuf::from(home).join("scans/canal_s0");
    std::fs::create_dir_all(&dir).expect("create out dir");
    dir
}

fn check_intersections(mesh: &IndexedMesh) -> usize {
    let params = IntersectionParams::default();
    detect_self_intersections(mesh, &params).intersection_count
}

/// Texture-recovery metric (m). Detrends frenulum-side vertices in the
/// stimulation zone (`t ∈ [0.22, 0.58]`, `cos θ ≥ 0.8`) against the
/// smooth base radius and reports the peak |residual| + RMS + sample
/// count. A surviving 1.5 mm texture shows peak ≈ 1.5 mm; an obliterated
/// one shows ≈ 0.
fn measure_texture_recovery(mesh_m: &IndexedMesh, sdf: &CanalProbeSdf) -> (f64, f64, usize) {
    let mut residuals: Vec<f64> = Vec::new();
    for v in &mesh_m.vertices {
        let z = v.z;
        let t = z / sdf.length_m;
        if !(0.22..=0.58).contains(&t) {
            continue;
        }
        let r_xy = v.x.hypot(v.y);
        if r_xy < 1.0e-9 {
            continue;
        }
        let cos_theta = v.x.mul_add(sdf.frenulum_dir.x, v.y * sdf.frenulum_dir.y) / r_xy;
        if cos_theta < 0.8 {
            continue;
        }
        let r_base = sdf.base_radius_at(z, cos_theta);
        residuals.push(r_xy - r_base);
    }
    let n = residuals.len();
    let peak = residuals.iter().fold(0.0_f64, |a, &r| a.max(r.abs()));
    let rms = if n > 0 {
        (residuals.iter().map(|r| r * r).sum::<f64>() / n as f64).sqrt()
    } else {
        0.0
    };
    (peak, rms, n)
}

// ────────────────────────────────────────────────────────────────────
// Zone profiles
// ────────────────────────────────────────────────────────────────────

/// Standard zone profile (entry ring → clearance chamber → stim → taper)
/// as radius fractions of `r_p`, per recon §3. Used for the
/// resolution / clean-mesh sweep.
fn standard_stations() -> Vec<(f64, f64)> {
    vec![
        (0.00, 0.45), // entry ring — corona catch (tight)
        (0.10, 0.45),
        (0.14, 0.95), // clearance chamber — let corona clear (open)
        (0.18, 0.95),
        (0.22, 0.85), // stimulation zone (base)
        (0.60, 0.85),
        (0.92, 0.55), // collapsing taper (below r_p)
        (1.00, 0.55),
    ]
}

/// Deliberate-undercut profile for the pull-out test artifact: a neck,
/// then a chamber wider than the neck, then a neck again. Chamber/neck
/// radius ratio = 0.78 / 0.55 = 1.42 (in the recon's 1.3–1.5 target).
/// Axisymmetric (asymmetry off) — the pull-out failure mode is the
/// chamber bulge passing through the neck, independent of azimuth.
fn undercut_stations() -> Vec<(f64, f64)> {
    vec![
        (0.00, 0.55), // mouth / base
        (0.30, 0.55), // neck before chamber
        (0.35, 0.55),
        (0.45, 0.78), // chamber — wider than the neck (the undercut)
        (0.55, 0.78),
        (0.65, 0.55), // neck after chamber
        (1.00, 0.55), // terminal
    ]
}

const UNDERCUT_NECK_FRAC: f64 = 0.55;
const UNDERCUT_CHAMBER_FRAC: f64 = 0.78;

// ────────────────────────────────────────────────────────────────────
// The probe
// ────────────────────────────────────────────────────────────────────

#[test]
#[ignore = "S0 canal probe — run manually with --ignored"]
fn s0_canal_probe() {
    let out = out_dir();
    let frenulum_dir = Vector3::new(0.0, 1.0, 0.0);

    // ── Q-resolution + Q-meshes-clean ─────────────────────────────────
    // Full textured + asymmetric canal across the mesher sweep.
    let canal = CanalProbeSdf {
        length_m: PROBE_LENGTH_M,
        base_radius_m: PROBE_BASE_RADIUS_M,
        stations: standard_stations(),
        frenulum_dir,
        frenulum_factor: FRENULUM_FACTOR,
        dorsal_factor: DORSAL_FACTOR,
        texture_amp_m: PROBE_TEXTURE_AMP_M,
        texture_pitch_m: PROBE_TEXTURE_PITCH_M,
        textured: true,
        asymmetric: true,
    };
    let solid = Solid::from_sdf(canal.clone(), canal.bounds());

    eprintln!("\n=== S0 canal probe — textured + asymmetric canal ===");
    eprintln!(
        "  L = {:.0} mm, base r_p = {:.1} mm, texture A = {:.1} mm @ pitch {:.1} mm",
        PROBE_LENGTH_M * 1000.0,
        PROBE_BASE_RADIUS_M * 1000.0,
        PROBE_TEXTURE_AMP_M * 1000.0,
        PROBE_TEXTURE_PITCH_M * 1000.0,
    );
    eprintln!(
        "  D-section: frenulum {:.2}× / dorsal {:.2}× r_p\n",
        FRENULUM_FACTOR, DORSAL_FACTOR
    );

    eprintln!(
        "  mesher           | cell (mm) | wall_s | faces  | verts  | self-int | tex peak (mm) | tex rms (mm) | n_samp"
    );
    eprintln!(
        "  ---------------- | --------- | ------ | ------ | ------ | -------- | ------------- | ------------ | ------"
    );

    let cell_sizes_m = [0.003, 0.001, 0.0005];
    for &cell in cell_sizes_m.iter() {
        let start = Instant::now();
        let mesh_m = mesh_uniform(&solid, cell);
        let elapsed = start.elapsed();

        let (tex_peak, tex_rms, n_samp) = measure_texture_recovery(&mesh_m, &canal);
        let n_int = check_intersections(&mesh_m);

        let mut mesh_mm = mesh_m;
        scale_in_place(&mut mesh_mm, METERS_TO_MM);
        let fname = format!("canal_textured_mc_{}um.stl", (cell * 1.0e6).round() as i64);
        save_stl(&mesh_mm, out.join(&fname), true).expect("save_stl");

        eprintln!(
            "  uniform MC       | {:>9.3} | {:>5.2}s | {:>6} | {:>6} | {:>8} | {:>13.3} | {:>12.3} | {:>6}",
            cell * 1000.0,
            elapsed.as_secs_f64(),
            mesh_mm.faces.len(),
            mesh_mm.vertices.len(),
            n_int,
            tex_peak * 1000.0,
            tex_rms * 1000.0,
            n_samp,
        );
    }

    // Adaptive dual-contouring pass. mesh_adaptive returns an
    // AttributedMesh in METERS; convert to IndexedMesh by copying both
    // fields (same shapes per recon §13.1).
    let adaptive_tol_m = 0.0005;
    let start = Instant::now();
    let am = solid.mesh_adaptive(adaptive_tol_m);
    let elapsed = start.elapsed();
    let mesh_m = am.geometry;
    let (tex_peak, tex_rms, n_samp) = measure_texture_recovery(&mesh_m, &canal);
    let n_int = check_intersections(&mesh_m);
    let mut mesh_mm = mesh_m;
    scale_in_place(&mut mesh_mm, METERS_TO_MM);
    save_stl(&mesh_mm, out.join("canal_textured_adaptive.stl"), true).expect("save_stl");
    eprintln!(
        "  adaptive DC      | tol={:>5.3} | {:>5.2}s | {:>6} | {:>6} | {:>8} | {:>13.3} | {:>12.3} | {:>6}",
        adaptive_tol_m * 1000.0,
        elapsed.as_secs_f64(),
        mesh_mm.faces.len(),
        mesh_mm.vertices.len(),
        n_int,
        tex_peak * 1000.0,
        tex_rms * 1000.0,
        n_samp,
    );

    eprintln!(
        "\n  Q-resolution: texture peak ≈ {:.1} mm means the {:.1} mm rib survives;",
        PROBE_TEXTURE_AMP_M * 1000.0,
        PROBE_TEXTURE_AMP_M * 1000.0
    );
    eprintln!("  peak ≈ 0 means the mesher obliterated it. Compare across rows above.");

    // ── Q-pull-out (the showstopper) ──────────────────────────────────
    // Deliberate undercut: chamber/neck ratio ~1.42. Printed + cast in
    // scrap silicone by the workshop to bound how aggressive S4's
    // chambers/suction can be.
    let undercut = CanalProbeSdf {
        length_m: PROBE_LENGTH_M,
        base_radius_m: PROBE_BASE_RADIUS_M,
        stations: undercut_stations(),
        frenulum_dir,
        frenulum_factor: FRENULUM_FACTOR,
        dorsal_factor: DORSAL_FACTOR,
        texture_amp_m: PROBE_TEXTURE_AMP_M,
        texture_pitch_m: PROBE_TEXTURE_PITCH_M,
        textured: false, // axisymmetric — isolate the undercut variable
        asymmetric: false,
    };
    let undercut_solid = Solid::from_sdf(undercut.clone(), undercut.bounds());
    let undercut_cell_m = 0.0005;
    let start = Instant::now();
    let mut undercut_mesh = mesh_uniform(&undercut_solid, undercut_cell_m);
    let elapsed = start.elapsed();
    let n_int = check_intersections(&undercut_mesh);
    scale_in_place(&mut undercut_mesh, METERS_TO_MM);
    let undercut_path = out.join("canal_probe.stl");
    save_stl(&undercut_mesh, &undercut_path, true).expect("save_stl");

    let neck_r_mm = PROBE_BASE_RADIUS_M * UNDERCUT_NECK_FRAC * 1000.0;
    let chamber_r_mm = PROBE_BASE_RADIUS_M * UNDERCUT_CHAMBER_FRAC * 1000.0;
    let ratio = UNDERCUT_CHAMBER_FRAC / UNDERCUT_NECK_FRAC;

    eprintln!("\n=== Q-pull-out: deliberate-undercut pull-out test artifact ===");
    eprintln!(
        "  neck radius    = {:.2} mm  (Ø {:.2} mm)",
        neck_r_mm,
        neck_r_mm * 2.0
    );
    eprintln!(
        "  chamber radius = {:.2} mm  (Ø {:.2} mm)",
        chamber_r_mm,
        chamber_r_mm * 2.0
    );
    eprintln!(
        "  UNDERCUT RATIO = {:.3}  (chamber ÷ neck; recon target 1.3–1.5)",
        ratio
    );
    eprintln!(
        "  mesh: {} faces, {} verts, self-int = {}, MC @ {:.1} mm, {:.2}s",
        undercut_mesh.faces.len(),
        undercut_mesh.vertices.len(),
        n_int,
        undercut_cell_m * 1000.0,
        elapsed.as_secs_f64(),
    );
    eprintln!("  saved: {}", undercut_path.display());
    eprintln!("\n  outputs saved to: {}\n", out.display());

    // Sanity assertions — the probe itself must be sound for its numbers
    // to mean anything.
    assert!(
        !undercut_mesh.faces.is_empty() && !undercut_mesh.vertices.is_empty(),
        "undercut probe meshed empty"
    );
}
