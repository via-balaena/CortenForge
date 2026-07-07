//! Rung 7 of the geometry-fidelity ladder — **validate the assembled L4–L5
//! Functional Spinal Unit against the biomechanics literature** (the ladder's
//! summit, north-star #1).
//!
//! Rungs 4b/5/6 each proved one FSU structure in isolation: facet contact
//! (`ShapeConcave`), ligaments (`<spatial>` tendons), and the bonded soft disc
//! (`BondedSandwich`). This rung ASSEMBLES them and measures the *whole segment's*
//! flexion/extension moment–rotation response, then compares the range of motion
//! (ROM) and stiffness to the published L4–L5 corridor (Panjabi/White; Yamamoto
//! 1989; Wilke standardized in-vitro data) — **match-or-REPORT: a measured gap is a
//! valid result, not a failure.**
//!
//! ## Architecture — superposition of restoring moments (not a monolithic sim)
//!
//! The three structures live in two coordinate frames and two solvers, and each is
//! already a *prescribed-configuration → restoring-wrench* readout. So the segment's
//! response is their **superposition**: at each imposed flexion angle `θ` about a
//! single shared axis through a single shared pivot, sum
//! `M(θ) = M_disc(θ) + M_ligament(θ)` (both about that axis, in N·m). This is exact
//! for a rigid kinematic prescription — no structure feels the others' forces because
//! the configuration is *imposed*, not solved to equilibrium — and it is the standard
//! "impose rotation, measure reaction moment" inverse of the in-vitro pure-moment test.
//!
//! - **Shared pivot** = the disc geometric centre. In the disc's own (recentred,
//!   scaled) frame this is the origin; for the native-mm ligament/facet frame it is
//!   the disc mesh's native AABB centre — the same physical point, because all three
//!   `BodyParts3D` meshes share one native coordinate system.
//! - **Shared axis** = the medio-lateral (ML) axis. We adopt the disc's own
//!   field-derived principal ML axis (its widest AABB extent — never hardcoded) and
//!   cross-check it against the independent vertebral-frame ML (superior × posterior);
//!   they agree to a measured ~19° anatomical obliquity (the vertebral ML is tilted by
//!   lordosis + coarse body-centre localization), which is REPORTED, not assumed away.
//!
//! ## What is measured vs reported (build+measure, don't assert a proxy)
//!
//! The asserts are in two honestly-labelled tiers. **DIAGNOSTIC** (non-tautological —
//! they can catch a genuinely broken assembly):
//! - the **facet engagement is physiologically asymmetric** — it engages on exactly
//!   ONE rotation sense, which *defines* extension (the flexion sense is derived from
//!   this, not assumed from a hardcoded axis handedness);
//! - the **neutral pose is force-free** (θ=0: ligaments at slack, facets clear);
//! - the disc bends **linearly** at small strain (`k_disc` agrees across two probe
//!   angles) and its bond **conserves** (`ΣF`, `ΣM ≈ 0`, rung-6 oracle).
//!
//! **WIRING GUARDS** (weaker — the restoring `M·θ < 0` and monotone `|M|(|θ|)` shape of
//! the superposed curve already *follow* from each part being validated in its own rung,
//! so these only catch a sign/unit error in the superposition here, not wrong physics).
//! They are asserted but not oversold.
//!
//! REPORTED (printed, soft — never fails the test): ROM at the physiologic moment,
//! secant stiffness, and the gap to the literature band; plus a **sensitivity sweep**
//! over the uncalibrated parameters (ligament `k`, disc `μ`, facet penalty `K`) — the
//! honest account of what the ROM depends on, since rung-5 `k` and rung-6 `μ` were
//! mechanism-proofs, not calibrated tissue values. The disc's headline contribution is
//! a small-strain LINEAR EXTRAPOLATION of `k_disc` across the sweep (the quasi-static
//! bond only converges at sub-degree strains), so the reported flexion ROM landing in
//! the literature band is a band-membership check, NOT a calibrated point-match — the
//! band spans the 7.5–10 N·m of the source studies, and the flexion ROM is dominated by
//! the uncalibrated ligament `k` (the sensitivity sweep makes this explicit).
//!
//! ## Scope / honesty
//!
//! Flexion/extension about ML only (the axis all three pieces already support).
//! Lateral bending + axial rotation are a follow-on; axial rotation in particular
//! needs a trustworthy *facet force* (rung-4b proved facet contact geometry, not a
//! force), which is why the facet contribution here is qualitative (engagement) in
//! the headline and quantitative only inside the sensitivity band.
//!
//! The shared FSU geometry recipes (mesh load + oracle, the segmental frame, the
//! ligament attachment sites, the facet SDF grid) live in the `cf-fsu-geometry`
//! crate — this rung consumes them; only the literature harness, disc-bond model,
//! and superposition machinery below are rung-7-specific.
//!
//! Env-gated + license-clean like the other rungs: `#[ignore]` + `$CF_L4_STL` +
//! `$CF_L5_STL` + `$CF_DISC_STL` (`BodyParts3D` meshes are CC BY-SA, not committed).
//! Run with:
//!
//! ```text
//! CF_L4_STL=/path/FMA13075.stl CF_L5_STL=/path/FMA13076.stl CF_DISC_STL=/path/FMA16036.stl \
//!   cargo test -p sim-coupling --release \
//!   --test rung7_fsu_validation -- --ignored --nocapture
//! ```

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use std::sync::Arc;

use cf_fsu_geometry::{
    BODY_RADIUS, FACET_CELL, FACET_MAX_CONTACTS, SegmentFrame, extreme_vertex, facet_grid,
    load_from_env, oracle, segment_frame,
};
use cf_geometry::{Aabb, IndexedMesh};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_core::sdf::compute_shape_contact;
use sim_core::{Pose, SdfContact, SdfGrid, ShapeConcave};
use sim_coupling::BondedSandwich;
use sim_mjcf::load_model;
use sim_soft::{
    Aabb3, MaterialField, Mesh, MeshingHints, SdfMeshedTetMesh, Vec3, pick_vertices_by_predicate,
};

// ── Literature corridor (facts, not copyrightable): representative in-vitro L4–L5
// segmental ROM at a physiologic moment. Yamamoto 1989 (10 N·m): flexion 5.8°,
// extension 3.5°; Panjabi/White & Wilke standardized (7.5 N·m) sit in the same band.
// The corridors below are deliberately widened to span the 7.5–10 N·m range of the
// source studies (ROM grows with the applied moment), since we probe at 7.5 N·m; a
// point-match is not claimed, only whether our ROM lands inside this band. ──
const PHYSIOLOGIC_MOMENT: f64 = 7.5; // N·m
const LIT_FLEXION_DEG: (f64, f64) = (5.0, 8.5); // one-direction flexion ROM corridor
const LIT_EXTENSION_DEG: (f64, f64) = (2.5, 5.5); // one-direction extension ROM corridor

// ── Disc scene constants (SI metres; recentred+scaled — rung-6c discipline). ──
const SCALE: f64 = 1.0e-3; // mm → m
const CELL: f64 = 0.003; // tet lattice spacing (m)
const PAD: f64 = 0.0015; // lattice padding beyond the disc AABB (m)
const BAND_FRAC: f64 = 0.18; // endplate band = 18% of the disc SI extent
const H_BOX: f64 = 0.006; // rigid-vertebra box half-thickness (m)
const STATIC_DT: f64 = 1.0e3; // quasi-static disc
const LOWER: usize = 1;
const UPPER: usize = 2;
const DISC_PROBE_DEG: [f64; 2] = [0.5, 0.86]; // small angles: keep the tets in their SPD region
const DEFAULT_MU: f64 = 1.0e5; // disc Neo-Hookean shear modulus (Pa) — the rung-6c fallback

// ── Ligament / facet constants (native mm frame). ──
const K_LIG: f64 = 20.0; // tendon stiffness N/mm (rung-5, uncalibrated)
const K_FACET: f64 = 200.0; // facet penalty N/mm (uncalibrated; sensitivity only)

// ─────────────────────────── DISC SUBSYSTEM (scaled SI) ───────────────────────────

/// The bonded disc plus the geometry the flexion probe needs.
struct DiscModel {
    sandwich: BondedSandwich<SdfMeshedTetMesh>,
    ml_scaled: Vector3<f64>,    // ML direction in the (unrotated) disc frame
    rest_up: Vec3,              // superior box body origin at rest
    center_native: Point3<f64>, // disc AABB centre in native mm (the shared pivot)
}

fn disc_mjcf(c_inf: Vec3, c_sup: Vec3) -> String {
    let lo = c_inf - Vec3::z() * H_BOX;
    let hi = c_sup + Vec3::z() * H_BOX;
    format!(
        r#"<mujoco><option gravity="0 0 0" timestep="0.001"/><worldbody>
    <body name="inf" pos="{lx} {ly} {lz}"><freejoint/><geom type="box" size="0.025 0.025 {h}" mass="0.05"/></body>
    <body name="sup" pos="{ux} {uy} {uz}"><freejoint/><geom type="box" size="0.025 0.025 {h}" mass="0.05"/></body>
    </worldbody></mujoco>"#,
        lx = lo.x,
        ly = lo.y,
        lz = lo.z,
        ux = hi.x,
        uy = hi.y,
        uz = hi.z,
        h = H_BOX,
    )
}

/// Load + tet-mesh the real disc at shear modulus `mu`, bond it between two field-posed
/// boxes, and capture the shared pivot (disc AABB centre, native mm) + the ML direction.
fn build_disc(mu: f64) -> DiscModel {
    let mut mesh = load_from_env("CF_DISC_STL").expect("load disc mesh");
    let bbox0 = Aabb::from_points(mesh.vertices.iter());
    let center_native = Point3::from(bbox0.min.coords + (bbox0.max - bbox0.min) * 0.5);
    for v in &mut mesh.vertices {
        *v = Point3::from((v.coords - center_native.coords) * SCALE);
    }
    let sdf = oracle(&mesh).expect("disc oracle");
    let bbox = Aabb::from_points(mesh.vertices.iter());
    // Derive the disc's principal axes from its AABB extents (no axis on faith,
    // rung-6c discipline): SI = thinnest, ML (the flexion/extension axis) = widest.
    let span = bbox.max - bbox.min;
    assert!(
        span.z < span.x && span.z < span.y,
        "disc SI (thinnest) must be native z"
    );
    let spans = [span.x, span.y, span.z];
    let widest = (0..3)
        .max_by(|&a, &b| spans[a].total_cmp(&spans[b]))
        .unwrap();
    let mut ml_scaled = Vector3::zeros();
    ml_scaled[widest] = 1.0; // the disc's widest principal extent = ML
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(bbox.min.x - PAD, bbox.min.y - PAD, bbox.min.z - PAD),
            Vec3::new(bbox.max.x + PAD, bbox.max.y + PAD, bbox.max.z + PAD),
        ),
        cell_size: CELL,
        material_field: Some(MaterialField::uniform(mu, 4.0 * mu)),
    };
    let tet = SdfMeshedTetMesh::from_sdf(&sdf, &hints).expect("disc meshes");
    let (lo_z, hi_z) = (bbox.min.z, bbox.max.z);
    let band = BAND_FRAC * (hi_z - lo_z);
    let inferior = pick_vertices_by_predicate(&tet, |p| p.z < lo_z + band);
    let superior = pick_vertices_by_predicate(&tet, |p| p.z > hi_z - band);
    assert!(
        !inferior.is_empty() && !superior.is_empty(),
        "endplate bands non-empty"
    );
    let cen = |vs: &[u32]| {
        let s: Vec3 = vs.iter().map(|&v| tet.positions()[v as usize]).sum();
        s / vs.len() as f64
    };
    let (c_inf, c_sup) = (cen(&inferior), cen(&superior));
    println!(
        "disc(mu={mu:.0e}): {} tets, endplates {}+{} verts",
        tet.n_tets(),
        inferior.len(),
        superior.len()
    );
    let model = load_model(&disc_mjcf(c_inf, c_sup)).expect("disc FSU MJCF");
    let mut data = model.make_data();
    data.forward(&model).expect("disc forward");
    let rest_up = data.xpos[UPPER];
    let sandwich = BondedSandwich::from_tet_mesh(
        model, data, LOWER, UPPER, tet, inferior, superior, STATIC_DT,
    );
    DiscModel {
        sandwich,
        ml_scaled,
        rest_up,
        center_native,
    }
}

/// The disc's restoring response to a flexion `θ` about the ML axis through the disc
/// centre (origin in the scaled frame): the reaction moment on the superior vertebra
/// about that pivot, projected on ML (N·m), plus a conservation residual `‖ΣF‖`+`‖ΣM‖`
/// over both bonded faces (rung-6 oracle; ≈0 for a self-equilibrated field).
fn disc_flexion_moment(disc: &mut DiscModel, theta: f64) -> (f64, f64) {
    let axis = Unit::new_normalize(disc.ml_scaled);
    let rot = UnitQuaternion::from_axis_angle(&axis, theta);
    // rotate the superior box about the origin (disc centre); the bonded endplate
    // follows rigidly, so the two faces' relative rotation is exactly θ.
    disc.sandwich.set_body_pose(UPPER, rot * disc.rest_up, rot);
    disc.sandwich.probe();
    let react = disc.sandwich.last_reaction();
    let targets = disc.sandwich.last_targets();
    let at = |i: usize, s: &[f64]| Vec3::new(s[3 * i], s[3 * i + 1], s[3 * i + 2]);
    // moment on the superior vertebra about the origin (disc centre).
    let mut m_up = Vec3::zeros();
    for &v in disc.sandwich.upper_face() {
        let i = v as usize;
        m_up += at(i, targets).cross(&at(i, react));
    }
    // conservation over BOTH faces (should vanish).
    let (mut f_tot, mut m_tot) = (Vec3::zeros(), Vec3::zeros());
    for &v in disc
        .sandwich
        .lower_face()
        .iter()
        .chain(disc.sandwich.upper_face())
    {
        let i = v as usize;
        let f = at(i, react);
        f_tot += f;
        m_tot += at(i, targets).cross(&f);
    }
    (m_up.dot(&disc.ml_scaled), f_tot.norm() + m_tot.norm())
}

// ─────────────────────────── LIGAMENT SUBSYSTEM (native mm) ───────────────────────────

/// Build the 2-body flexion hinge + 2 ligaments (anterior ALL + posterior interspinous)
/// as pull-only `<spatial>` tendons, hinged about the shared ML axis through `pivot`.
fn build_ligament_model(
    l4: &IndexedMesh,
    l5: &IndexedMesh,
    frame: &SegmentFrame,
    ml: Vector3<f64>,
    pivot: Point3<f64>,
    k_lig: f64,
) -> sim_core::Model {
    let (b4, b5, post) = (frame.b4, frame.b5, frame.posterior);
    let attach = |m: &IndexedMesh, o: Point3<f64>, d: Vector3<f64>, r: f64| {
        extreme_vertex(m, o, d, ml, r).expect("ligament attachment site (no vertex qualifies)")
    };
    let all4 = attach(l4, b4, -post, BODY_RADIUS);
    let all5 = attach(l5, b5, -post, BODY_RADIUS);
    let isp4 = attach(l4, b4, post, f64::INFINITY);
    let isp5 = attach(l5, b5, post, f64::INFINITY);
    let slack = |a: Point3<f64>, b: Point3<f64>| (a - b).norm();
    let site = |p: Point3<f64>| format!("{} {} {}", p.x, p.y, p.z);
    let mjcf = format!(
        r#"<mujoco model="rung7_ligaments"><option gravity="0 0 0"/><worldbody>
      <body name="L5" pos="0 0 0"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <site name="l5a" pos="{a5}"/><site name="l5i" pos="{i5}"/></body>
      <body name="L4" pos="0 0 0"><joint name="flex" type="hinge" axis="{mx} {my} {mz}" pos="{pv}"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <site name="l4a" pos="{a4}"/><site name="l4i" pos="{i4}"/></body></worldbody>
      <tendon>
        <spatial name="ALL" stiffness="{k}" springlength="0 {sa}" damping="0" width="0.5"><site site="l5a"/><site site="l4a"/></spatial>
        <spatial name="ISP" stiffness="{k}" springlength="0 {si}" damping="0" width="0.5"><site site="l5i"/><site site="l4i"/></spatial>
      </tendon></mujoco>"#,
        a5 = site(all5),
        i5 = site(isp5),
        a4 = site(all4),
        i4 = site(isp4),
        mx = ml.x,
        my = ml.y,
        mz = ml.z,
        pv = site(pivot),
        k = k_lig,
        sa = slack(all4, all5),
        si = slack(isp4, isp5),
    );
    load_model(&mjcf).expect("ligament model")
}

/// Ligament restoring moment about the hinge (ML) axis at flexion `θ` (rad), in N·m.
/// `qfrc_spring` is the generalized force conjugate to the hinge angle = moment about
/// the ML axis in N·mm (native mm, `k` in N/mm) → ×1e-3 → N·m.
fn ligament_moment(model: &sim_core::Model, theta: f64) -> f64 {
    let mut data = model.make_data();
    data.qpos[0] = theta;
    data.forward(model).expect("ligament forward");
    data.qfrc_spring[0] * 1e-3
}

// ─────────────────────────── FACET SUBSYSTEM (native mm) ───────────────────────────

/// Facet response at flexion `θ`: L4 rotated by `θ` about the ML axis through `pivot`,
/// L5 fixed. Returns (number of engaged facet contacts, penalty restoring moment about
/// ML in N·m). Contacts fire only when the articular surfaces overlap → zero in flexion,
/// engaging in extension. The penalty moment uses the uncalibrated `k_facet` (N/mm), so
/// it is reported for sensitivity only, never in the headline sum.
fn facet_response(
    g4: &Arc<SdfGrid>,
    g5: &Arc<SdfGrid>,
    pivot: Point3<f64>,
    ml: Vector3<f64>,
    theta: f64,
    k_facet: f64,
) -> (usize, f64) {
    let r = UnitQuaternion::from_axis_angle(&Unit::new_normalize(ml), theta);
    let pose_a = Pose {
        position: Point3::from(pivot.coords - r * pivot.coords),
        rotation: r,
    };
    let pose_b = Pose {
        position: Point3::origin(),
        rotation: UnitQuaternion::identity(),
    };
    let cs: Vec<SdfContact> = compute_shape_contact(
        &ShapeConcave::new(Arc::clone(g4)),
        &pose_a,
        &ShapeConcave::new(Arc::clone(g5)),
        &pose_b,
        FACET_CELL,
        FACET_MAX_CONTACTS,
    );
    let mut m = Vec3::zeros();
    let mut engaged = 0;
    for c in &cs {
        if c.penetration <= 0.0 {
            continue;
        }
        engaged += 1;
        let f = c.normal * (k_facet * c.penetration);
        m += (c.point - pivot).cross(&f);
    }
    (engaged, m.dot(&ml) * 1e-3)
}

// ─────────────────────────── ROM / stiffness readouts ───────────────────────────

/// Interpolate the angle (deg) at which `|M|` first reaches `target` N·m on the side
/// of neutral selected by `sign` (+1 flexion, −1 extension). `None` if the curve never
/// crosses `target` over the swept range — which happens either because the segment is
/// too lax (peak `|M| < target`) or too stiff (`|M| > target` already at the smallest
/// swept angle); [`report_rom`] disambiguates the two.
fn rom_at(curve: &[(f64, f64)], target: f64, sign: f64) -> Option<f64> {
    let mut pts: Vec<(f64, f64)> = curve
        .iter()
        .copied()
        .filter(|(d, _)| d * sign > 0.0)
        .map(|(d, m)| (d.abs(), m.abs()))
        .collect();
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    for w in pts.windows(2) {
        let ((d0, m0), (d1, m1)) = (w[0], w[1]);
        if (m0 - target) * (m1 - target) <= 0.0 && (m1 - m0).abs() > 1e-12 {
            return Some(d0 + (target - m0) * (d1 - d0) / (m1 - m0));
        }
    }
    None
}

/// Report the ROM on one side of neutral against its corridor. When the curve never
/// reaches the physiologic moment we distinguish the two ways that happens — the
/// segment is either too LAX (peak `|M|` below target over the whole swept range → ROM
/// beyond it) or too STIFF (already above target at the smallest swept angle → ROM
/// below it) — by reporting the peak `|M|` and the angle it occurred at. `note` explains
/// a known cause (e.g. facet force excluded from the headline).
fn report_rom(label: &str, curve: &[(f64, f64)], sign: f64, corridor: (f64, f64), note: &str) {
    let (lo, hi) = corridor;
    if let Some(deg) = rom_at(curve, PHYSIOLOGIC_MOMENT, sign) {
        let verdict = if deg >= lo && deg <= hi {
            "within lit band".to_string()
        } else {
            let nearest = if deg < lo { lo } else { hi };
            format!("GAP {:+.1}° from [{lo:.1},{hi:.1}]", deg - nearest)
        };
        println!("[{label}] ROM = {deg:.2}°  (lit band [{lo:.1},{hi:.1}]°)  -> {verdict}");
        return;
    }
    // Never crossed the target: report the peak achieved so lax vs stiff is unambiguous.
    let side: Vec<(f64, f64)> = curve
        .iter()
        .copied()
        .filter(|(d, _)| d * sign > 0.0)
        .collect();
    let (pk_deg, pk_m) = side
        .iter()
        .copied()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .unwrap_or((0.0, 0.0));
    if pk_m.abs() < PHYSIOLOGIC_MOMENT {
        println!(
            "[{label}] ROM > swept range: too LAX — peaked at only {:.2} N·m at {:.1}° (< {PHYSIOLOGIC_MOMENT}); {note}",
            pk_m.abs(),
            pk_deg.abs()
        );
    } else {
        println!(
            "[{label}] ROM < smallest swept angle: too STIFF — already {:.2} N·m at {:.1}°",
            pk_m.abs(),
            pk_deg.abs()
        );
    }
}

#[test]
#[ignore = "needs $CF_L4_STL $CF_L5_STL $CF_DISC_STL (BodyParts3D, CC BY-SA, not committed)"]
fn l4_l5_fsu_segmental_response_vs_literature() {
    println!("\n=== RUNG 7 — assembled L4–L5 FSU: flexion/extension response vs literature ===");

    // ── Shared frame from the two vertebrae (native mm) ──
    let l4 = load_from_env("CF_L4_STL").expect("load L4 mesh");
    let l5 = load_from_env("CF_L5_STL").expect("load L5 mesh");
    let o4 = oracle(&l4).expect("L4 oracle");
    let o5 = oracle(&l5).expect("L5 oracle");
    let frame = segment_frame(&l4, &l5, &o4, &o5).expect("segment frame");
    println!(
        "[frame] ML = ({:.3},{:.3},{:.3})",
        frame.ml.x, frame.ml.y, frame.ml.z
    );

    // ── Disc: build, verify the ML axes agree, measure the small-strain bending
    //    stiffness (linearity across two probe angles) + conservation ──
    let mut disc = build_disc(DEFAULT_MU);
    let pivot = disc.center_native; // shared pivot (native mm)
    // The shared flexion/extension axis = the disc's own field-derived principal ML axis
    // (its widest extent). Cross-check it against the independent vertebral-frame ML
    // (superior × posterior): they agree to an anatomical obliquity — the vertebral ML is
    // tilted by the lordotic body-centre vector + coarse body-centre localization, so we
    // adopt the cleaner disc axis and REPORT the disagreement rather than assume they match.
    let flex_axis = disc.ml_scaled;
    let ml_align = frame.ml.dot(&flex_axis).abs();
    println!(
        "[frame] flexion axis = disc ML ({:.2},{:.2},{:.2}); vertebral-ML agreement = {ml_align:.4} \
         (obliquity {:.1}°)",
        flex_axis.x,
        flex_axis.y,
        flex_axis.z,
        ml_align.clamp(-1.0, 1.0).acos().to_degrees()
    );
    assert!(
        ml_align > 0.9,
        "the disc ML and vertebral ML must roughly agree (sanity: axis is left-right)"
    );

    let mut k_disc = 0.0;
    let mut ks = Vec::new();
    for &deg in &DISC_PROBE_DEG {
        let th = deg.to_radians();
        let (m, resid) = disc_flexion_moment(&mut disc, th);
        assert!(
            resid < 1e-8,
            "disc bond must conserve (‖ΣF‖+‖ΣM‖ = {resid:.2e})"
        );
        let k = m / th;
        println!(
            "[disc] θ={deg:.2}°  M={m:+.5} N·m  k=M/θ={k:+.4} N·m/rad  (conservation {resid:.1e})"
        );
        ks.push(k);
        k_disc = k;
    }
    let k_spread = (ks[0] - ks[1]).abs() / ks[1].abs().max(1e-9);
    assert!(
        k_spread < 0.1,
        "disc must bend linearly at small strain (k spread {k_spread:.2})"
    );
    assert!(
        k_disc < 0.0,
        "disc bending must be restoring (k_disc<0), got {k_disc:+.4}"
    );
    println!(
        "[disc] linear bending stiffness k_disc = {k_disc:+.4} N·m/rad = {:+.5} N·m/deg",
        k_disc.to_radians()
    );

    // ── Ligaments + facets (native frame, shared pivot/axis) ──
    let lig = build_ligament_model(&l4, &l5, &frame, flex_axis, pivot, K_LIG);
    let (g4, g5) = (facet_grid(&l4, &o4), facet_grid(&l5, &o5));

    // ── Which rotation sense is flexion, DERIVED FROM THE FIELD (no handedness on
    //    faith — the ladder's anti-pattern): the facets open in flexion and engage in
    //    extension, so probe both senses and let the engagement asymmetry define the
    //    labels AND assert it. Robust to a mirrored / differently-handed mesh. ──
    let probe = 6.0_f64.to_radians();
    let (n_pos, _) = facet_response(&g4, &g5, pivot, flex_axis, probe, K_FACET);
    let (n_neg, _) = facet_response(&g4, &g5, pivot, flex_axis, -probe, K_FACET);
    assert!(
        (n_pos == 0) ^ (n_neg == 0),
        "facets must engage on exactly ONE rotation sense (the extension side): +{n_pos} / −{n_neg}"
    );
    let flexion_sign = if n_pos == 0 { 1.0 } else { -1.0 }; // flexion = the sense the facets OPEN
    println!(
        "[facets] engagement asymmetry: +6°→{n_pos} contacts, −6°→{n_neg}; flexion = {flexion_sign:+.0}·θ (facets open)"
    );

    // ── Neutral pose must be force-free (the module doc's invariant): at θ=0 the
    //    ligaments sit at slack and the facets are clear. Non-tautological — a nonzero
    //    slack offset or a mis-placed site would pre-tension the segment and skew the
    //    whole ROM baseline. (The disc term is k_disc·0 ≡ 0 and needs no probe.) ──
    let m_lig0 = ligament_moment(&lig, 0.0);
    let (n_facet0, _) = facet_response(&g4, &g5, pivot, flex_axis, 0.0, K_FACET);
    assert!(
        m_lig0.abs() < 1e-6,
        "neutral ligament moment must vanish (sites at slack): {m_lig0:.2e}"
    );
    assert_eq!(n_facet0, 0, "facets must be clear at the neutral pose");

    // ── Sweep; superpose disc + ligaments (headline), track facet engagement. ──
    println!(
        "\n{:>8} {:>10} {:>10} {:>11} {:>9} {:>11}",
        "θ(deg)", "M_disc", "M_lig", "M(head)", "nFacet", "M_facet"
    );
    let sweep = [-8.0_f64, -6.0, -4.0, -2.0, -1.0, 1.0, 2.0, 4.0, 6.0, 8.0];
    let mut headline: Vec<(f64, f64)> = Vec::new();
    let mut facet_flexion_contacts = 0usize;
    let mut facet_first_extension: Option<f64> = None;
    for &deg in &sweep {
        let th = deg.to_radians();
        let m_disc = k_disc * th; // small-strain linear extrapolation of the disc bending stiffness
        let m_lig = ligament_moment(&lig, th);
        let m_head = m_disc + m_lig;
        let (n_facet, m_facet) = facet_response(&g4, &g5, pivot, flex_axis, th, K_FACET);
        println!(
            "{deg:>8.1} {m_disc:>+10.4} {m_lig:>+10.4} {m_head:>+11.4} {n_facet:>9} {m_facet:>+11.4}"
        );
        headline.push((deg, m_head));
        if deg * flexion_sign > 0.0 {
            facet_flexion_contacts += n_facet; // flexion side — facets must stay open
        } else if n_facet > 0 {
            let a = deg.abs();
            facet_first_extension = Some(facet_first_extension.map_or(a, |o| o.min(a)));
        }
    }

    // ── Checks, two tiers, honestly labelled. DIAGNOSTIC (non-tautological — catch a
    //    genuinely broken assembly): the facet engagement asymmetry + the neutral
    //    force-free pose (both asserted above) + the disc conservation & linearity
    //    (asserted at build). WIRING GUARDS (weaker — the restoring/monotone shape of
    //    the SUM already follows from each part being validated in its own rung, so
    //    these only catch a sign/unit error in the superposition HERE, not wrong
    //    physics): ──
    for &(deg, m) in &headline {
        assert!(
            m * deg < 0.0,
            "superposed moment must oppose the displacement at θ={deg} (M={m:+.4}) — sign/unit wiring guard"
        );
    }
    for side in [1.0_f64, -1.0] {
        let mut pts: Vec<(f64, f64)> = headline
            .iter()
            .copied()
            .filter(|(d, _)| d * side > 0.0)
            .map(|(d, m)| (d.abs(), m.abs()))
            .collect();
        pts.sort_by(|a, b| a.0.total_cmp(&b.0));
        for w in pts.windows(2) {
            assert!(
                w[1].1 >= w[0].1 - 1e-9,
                "superposed |M| must be monotone in |θ| (side {side}) — wiring guard"
            );
        }
    }
    assert_eq!(
        facet_flexion_contacts, 0,
        "facets must NOT engage on the flexion side — physiological asymmetry"
    );
    assert!(
        facet_first_extension.is_some(),
        "facets must engage on the extension side — the rung-4b articular geometry"
    );
    println!(
        "\n[facets] flexion-side contacts = {facet_flexion_contacts} (must be 0); \
         first engaged EXTENSION swept angle = {:.0}° (grid-limited — the true onset lies below this)",
        facet_first_extension.unwrap()
    );

    // ── REPORTED comparison (soft — a measured gap is a valid result, never fails). ──
    println!(
        "\n── L4–L5 segmental ROM at {PHYSIOLOGIC_MOMENT} N·m (headline = disc + ligaments, UNCALIBRATED) ──"
    );
    report_rom("flexion ", &headline, flexion_sign, LIT_FLEXION_DEG, "");
    report_rom(
        "extension",
        &headline,
        -flexion_sign,
        LIT_EXTENSION_DEG,
        "the physiological extension limiter is facet contact, excluded from the headline (see facet engagement above)",
    );
    println!(
        "[caveats] the disc term is a SMALL-STRAIN LINEAR EXTRAPOLATION (k_disc measured at ≤{:.2}°, \
         applied across the ±8° sweep — the real disc stiffens nonlinearly there); the flexion ROM is \
         DOMINATED by the uncalibrated ligament k (see sensitivity below); the literature band spans the \
         7.5–10 N·m of the source studies, so 'within band' is a band-membership check, NOT a calibrated \
         point-match.",
        DISC_PROBE_DEG[1]
    );
    // Secant stiffness where the curve reaches the physiologic moment (N·m/deg).
    if let Some(f) = rom_at(&headline, PHYSIOLOGIC_MOMENT, flexion_sign) {
        println!(
            "[stiffness] flexion secant ≈ {:.3} N·m/deg",
            PHYSIOLOGIC_MOMENT / f
        );
    }
    if let Some(e) = rom_at(&headline, PHYSIOLOGIC_MOMENT, -flexion_sign) {
        println!(
            "[stiffness] extension secant ≈ {:.3} N·m/deg",
            PHYSIOLOGIC_MOMENT / e
        );
    }

    // ── SENSITIVITY: what the uncalibrated params do to flexion ROM (the honest gap
    //    account). Ligament k and disc μ dominate; facet K is an extension-only band. ──
    println!("\n── SENSITIVITY (uncalibrated params → flexion ROM) ──");
    for k_lig in [10.0, 20.0, 40.0] {
        let m = build_ligament_model(&l4, &l5, &frame, flex_axis, pivot, k_lig);
        let curve: Vec<(f64, f64)> = sweep
            .iter()
            .map(|&d| {
                (
                    d,
                    k_disc * d.to_radians() + ligament_moment(&m, d.to_radians()),
                )
            })
            .collect();
        println!(
            "[k_lig={k_lig:>4.0} N/mm] flexion ROM = {:?}°",
            rom_at(&curve, PHYSIOLOGIC_MOMENT, flexion_sign)
        );
    }
    for mu in [DEFAULT_MU, 5.0 * DEFAULT_MU, 10.0 * DEFAULT_MU] {
        // reuse the already-built disc's k_disc for DEFAULT_MU; only rebuild (the test's
        // most expensive step) for the higher μ values.
        let kd = if (mu - DEFAULT_MU).abs() < f64::EPSILON {
            k_disc
        } else {
            let mut d = build_disc(mu);
            let (m_probe, _) = disc_flexion_moment(&mut d, DISC_PROBE_DEG[1].to_radians());
            m_probe / DISC_PROBE_DEG[1].to_radians()
        };
        let curve: Vec<(f64, f64)> = sweep
            .iter()
            .map(|&deg| {
                (
                    deg,
                    kd * deg.to_radians() + ligament_moment(&lig, deg.to_radians()),
                )
            })
            .collect();
        println!(
            "[mu={mu:>7.0e} Pa] k_disc = {kd:+.3} N·m/rad, flexion ROM = {:?}°",
            rom_at(&curve, PHYSIOLOGIC_MOMENT, flexion_sign)
        );
    }
    // Facet extension penalty band. The ENGAGEMENT (contact count) is the solid,
    // rung-4b-grounded result; the penalty MAGNITUDE scales with the uncalibrated
    // `k_facet`, and its SIGN/line-of-action from the raw grid contact normals is not
    // validated as restoring — which is exactly why the facet term is kept out of the
    // headline sum and shown only as this magnitude band.
    let ext_deg = -flexion_sign * 6.0; // 6° into extension, whichever sense that is
    for k_facet in [100.0, 200.0, 400.0] {
        let (n, m) = facet_response(&g4, &g5, pivot, flex_axis, ext_deg.to_radians(), k_facet);
        println!(
            "[K_facet={k_facet:>4.0} N/mm] at {ext_deg}°: {n} contacts, |penalty moment| = {:.3} N·m (sign unvalidated)",
            m.abs()
        );
    }

    println!(
        "\n=== rung 7 done — assembled FSU characterized; gap to literature reported above ===\n"
    );
}
