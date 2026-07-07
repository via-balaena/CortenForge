//! Rung 5 of the geometry-fidelity ladder — **ligament tension on the L4–L5 FSU**.
//!
//! Rung 4b established the two-vertebra substrate (facet contact). This rung adds
//! the passive **ligaments** and proves the capability the FSU needs: a ligament
//! produces the correct *restoring* wrench when the segment is displaced, it is
//! strictly **pull-only** (a slackened ligament carries zero force), and it is
//! anchored at real, field-derived anatomical sites.
//!
//! ## What it is (and is NOT)
//!
//! Ligaments are modelled as MuJoCo **spatial tendons** — the engine's
//! first-class "tension member over a path between sites on two bodies" — with a
//! slack **deadband** (`springlength="0 L_slack"`) so force appears only when the
//! ligament is stretched past its natural length. This is the *isolated ligament
//! capability*: facet contact (rung 4b) and the bonded disc (rung 6) are NOT
//! present, so the measured wrench is unambiguously the ligaments'. The segment
//! is driven by a single **flexion/extension hinge** about the medio-lateral axis
//! through the disc center — one DOF whose generalized force IS the restoring
//! moment about that axis (no 6-DOF frame interpretation). Full coupled 6-DOF ROM
//! is rung 7.
//!
//! ## Measured, nothing asserted via a proxy (the ladder's recurring lesson)
//!
//! - The anatomical frame is **derived from the field**, no axis taken on faith:
//!   superior = L5→L4 body-center vector (body center = deepest interior point,
//!   as in rung 4b); posterior = body-center→mesh-centroid (the neural arch +
//!   processes pull the centroid posteriorly); medio-lateral = their cross.
//! - Attachment **sites** are field-derived surface points (anterior-most body
//!   point for the ALL; most-posterior near-midline point for the interspinous),
//!   each *asserted* to land in its expected region.
//! - The restoring moment is computed **by hand** from the tendon tensions and
//!   their moment arms (`M = Σ (r × F)·axis`), asserted to oppose the imposed
//!   angle, and then **cross-checked** against the engine's `qfrc_spring` (the
//!   Jᵀ mapping) — oracle matched to the system under test.
//!
//! Two ligaments (anterior ALL + posterior interspinous) bracket the segment so
//! pull-only restoring is shown in BOTH directions (flexion stretches one and
//! slackens the other; extension reverses it). The remaining ligaments (PLL,
//! ligamentum flavum, facet capsules) are rung-7 completeness, not this
//! capability proof.
//!
//! Env-gated + license-clean like the other rungs: `#[ignore]` + `$CF_L4_STL` +
//! `$CF_L5_STL` (`BodyParts3D` vertebrae are CC BY-SA, not committed). Run with:
//!
//! ```text
//! CF_L4_STL=/path/to/L4.stl CF_L5_STL=/path/to/L5.stl \
//!   cargo test -p cf-design-tests --release \
//!   --test rung5_ligament_tension -- --ignored --nocapture
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

use cf_fsu_geometry::{body_center, load_from_env, oracle};
use cf_geometry::IndexedMesh;
use nalgebra::{Point3, Vector3};
use sim_mjcf::load_model;

/// Tendon stiffness (N/mm). Model is kept in native millimetres, so tension is in
/// newtons and moment in N·mm — self-consistent with the hand oracle.
const K: f64 = 20.0;

/// Radius (mm) about a vertebral body center within which the anterior ALL site
/// is sought — large enough to contain a lumbar body's anterior rim, small
/// enough to exclude the spinous/transverse processes.
const BODY_RADIUS: f64 = 30.0;

/// Mean of the mesh vertices (the full-vertebra centroid — pulled posteriorly by
/// the neural arch/processes relative to the body center).
fn centroid(mesh: &IndexedMesh) -> Point3<f64> {
    let sum: Vector3<f64> = mesh.vertices.iter().map(|v| v.coords).sum();
    Point3::from(sum / mesh.vertices.len() as f64)
}

/// Orthonormal anatomical frame (all field-derived, no label taken on faith).
struct Frame {
    superior: Vector3<f64>,  // L5 body → L4 body
    posterior: Vector3<f64>, // body → mesh centroid, ⟂ superior
    lateral: Vector3<f64>,   // superior × posterior (medio-lateral)
}

/// Farthest surface vertex from `origin` along `dir`, restricted to vertices
/// within `near_lateral` of the mid-sagittal plane (small offset ALONG the
/// `lateral` axis, isolating a midline feature from the laterally-placed
/// transverse processes) AND within `max_radius` of `origin` (pass `INFINITY`
/// for a far feature like the spinous tip; a body-scale radius to force the
/// result onto the vertebral body rather than a distant process/osteophyte).
fn extreme_vertex(
    mesh: &IndexedMesh,
    origin: Point3<f64>,
    dir: Vector3<f64>,
    lateral: Vector3<f64>,
    near_lateral: f64,
    max_radius: f64,
) -> Point3<f64> {
    let mut best = origin;
    let mut best_proj = f64::MIN;
    for v in &mesh.vertices {
        let d = v - origin;
        if d.dot(&lateral).abs() > near_lateral || d.norm() > max_radius {
            continue; // off the mid-sagittal plane, or outside the target region
        }
        let proj = d.dot(&dir);
        if proj > best_proj {
            best_proj = proj;
            best = *v;
        }
    }
    best
}

#[test]
#[ignore = "needs local L4+L5 vertebra meshes via $CF_L4_STL/$CF_L5_STL (CC BY-SA, not committed)"]
fn l4_l5_ligament_tension_is_pull_only_and_restoring() {
    let l4 = load_from_env("CF_L4_STL").expect("load L4 mesh");
    let l5 = load_from_env("CF_L5_STL").expect("load L5 mesh");
    let o4 = oracle(&l4).expect("L4 oracle");
    let o5 = oracle(&l5).expect("L5 oracle");

    // ── Field-derived anatomical frame ──────────────────────────────────────
    let (b4, depth4) = body_center(&l4, &o4);
    let (b5, depth5) = body_center(&l5, &o5);
    assert!(
        depth4 < -5.0 && depth5 < -5.0,
        "body centers must be thick solid mass (depths {depth4:.1}, {depth5:.1})"
    );
    let superior = (b4 - b5).normalize();
    // posterior = body→centroid, orthogonalized against superior, averaged over
    // both vertebrae for a stable segment axis.
    let raw_post = (centroid(&l4) - b4) + (centroid(&l5) - b5);
    let posterior = (raw_post - raw_post.dot(&superior) * superior).normalize();
    let lateral = superior.cross(&posterior).normalize();
    let frame = Frame {
        superior,
        posterior,
        lateral,
    };
    println!(
        "[frame] superior=({:.2},{:.2},{:.2}) posterior=({:.2},{:.2},{:.2}) lateral=({:.2},{:.2},{:.2})",
        frame.superior.x,
        frame.superior.y,
        frame.superior.z,
        frame.posterior.x,
        frame.posterior.y,
        frame.posterior.z,
        frame.lateral.x,
        frame.lateral.y,
        frame.lateral.z,
    );
    // Orthonormality sanity.
    assert!(frame.superior.dot(&frame.posterior).abs() < 1e-9);
    assert!(frame.superior.dot(&frame.lateral).abs() < 1e-9);
    assert!(frame.posterior.dot(&frame.lateral).abs() < 1e-9);

    // ── Field-derived attachment sites ──────────────────────────────────────
    let midline_tol = 8.0; // mm off the mid-sagittal plane
    // Interspinous: most-posterior near-midline vertex (spinous tip) — a far
    // feature, so no radius bound.
    let isp4 = extreme_vertex(
        &l4,
        b4,
        frame.posterior,
        frame.lateral,
        midline_tol,
        f64::INFINITY,
    );
    let isp5 = extreme_vertex(
        &l5,
        b5,
        frame.posterior,
        frame.lateral,
        midline_tol,
        f64::INFINITY,
    );
    // ALL: most-anterior near-midline vertex WITHIN the body region — the radius
    // bound forces it onto the anterior body rim, not a distant anterior process
    // or osteophyte. `BODY_RADIUS` comfortably contains a lumbar body's anterior
    // rim (~half its AP depth from the center) while excluding the spinous tip.
    let all4 = extreme_vertex(
        &l4,
        b4,
        -frame.posterior,
        frame.lateral,
        midline_tol,
        BODY_RADIUS,
    );
    let all5 = extreme_vertex(
        &l5,
        b5,
        -frame.posterior,
        frame.lateral,
        midline_tol,
        BODY_RADIUS,
    );
    let post_off = |p: Point3<f64>, b: Point3<f64>| (p - b).dot(&frame.posterior);
    println!(
        "[sites] ALL: L4 posterior-offset={:.1}  L5={:.1}  |  ISP: L4={:.1}  L5={:.1}",
        post_off(all4, b4),
        post_off(all5, b5),
        post_off(isp4, b4),
        post_off(isp5, b5),
    );
    // The spinous (ISP) sites must be well posterior of the body; the ALL sites
    // anterior AND genuinely on the body (much nearer the body center than the
    // far spinous tips) — all measured, not assumed.
    assert!(
        post_off(isp4, b4) > 15.0 && post_off(isp5, b5) > 15.0,
        "interspinous sites must be well posterior of the body center"
    );
    assert!(
        post_off(all4, b4) < -5.0 && post_off(all5, b5) < -5.0,
        "ALL sites must be anterior of the body center"
    );
    assert!(
        (all4 - b4).norm() < post_off(isp4, b4) && (all5 - b5).norm() < post_off(isp5, b5),
        "ALL sites must lie on the vertebral body (nearer its center than the spinous tip), \
         not a distant anterior feature"
    );

    // Disc center + flexion hinge (medio-lateral axis through the disc).
    let disc = Point3::from(0.5 * (b4.coords + b5.coords));
    let axis = frame.lateral;

    // ── Build the 2-body + hinge + 2-tendon model ───────────────────────────
    // Both bodies at world origin carry their native-coordinate sites, so they
    // sit at their true anatomical relative pose at θ=0. L5 is the fixed base;
    // L4 rotates on a flexion hinge about `disc`. Slack = neutral length.
    let slack = |a: Point3<f64>, b: Point3<f64>| (a - b).norm();
    let slack_all = slack(all4, all5);
    let slack_isp = slack(isp4, isp5);
    let site = |p: Point3<f64>| format!("{} {} {}", p.x, p.y, p.z);
    let mjcf = format!(
        r#"
<mujoco model="rung5_ligaments">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="L5" pos="0 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <site name="l5_all" pos="{all5_p}"/>
      <site name="l5_isp" pos="{isp5_p}"/>
    </body>
    <body name="L4" pos="0 0 0">
      <joint name="flex" type="hinge" axis="{ax} {ay} {az}" pos="{dc}"/>
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <site name="l4_all" pos="{all4_p}"/>
      <site name="l4_isp" pos="{isp4_p}"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="ALL" stiffness="{K}" springlength="0 {slack_all}" damping="0" width="0.5">
      <site site="l5_all"/><site site="l4_all"/>
    </spatial>
    <spatial name="ISP" stiffness="{K}" springlength="0 {slack_isp}" damping="0" width="0.5">
      <site site="l5_isp"/><site site="l4_isp"/>
    </spatial>
  </tendon>
</mujoco>
"#,
        all5_p = site(all5),
        isp5_p = site(isp5),
        all4_p = site(all4),
        isp4_p = site(isp4),
        ax = axis.x,
        ay = axis.y,
        az = axis.z,
        dc = site(disc),
    );
    let model = load_model(&mjcf).expect("build L4-L5 ligament model");
    assert_eq!(model.ntendon, 2, "expected ALL + ISP tendons");
    let (t_all, t_isp) = (
        model
            .tendon_name_to_id
            .get("ALL")
            .copied()
            .expect("ALL tendon registered"),
        model
            .tendon_name_to_id
            .get("ISP")
            .copied()
            .expect("ISP tendon registered"),
    );

    // ── Sweep flexion/extension; one static evaluation per angle ────────────
    // Everything is read from a single `forward` per angle: the two tendon
    // tensions, the engine's ligament generalized moment (`qfrc_spring`), the
    // measured tendon lengths, and the hand-oracle moment `M = Σ (r × F)·axis`
    // recomputed from the world site positions.
    let deg2rad = std::f64::consts::PI / 180.0;
    let measure = |deg: f64| -> Sample {
        let mut data = model.make_data();
        data.qpos[0] = deg * deg2rad;
        data.forward(&model).expect("forward");
        // Tension magnitude = max(0, −ten_force) (deadband spring: force<0 in tension).
        let tension = |t: usize| (-data.ten_force[t]).max(0.0);
        let world = |name: &str| Point3::from(data.site_xpos[model.site_name_to_id[name]]);
        // Moment of one tendon about the hinge axis through `disc`: the L4 site is
        // pulled toward the L5 site, with moment arm (site_L4 − disc).
        let moment = |l4s: &str, l5s: &str, t: usize| {
            let (p4, p5) = (world(l4s), world(l5s));
            (p4 - disc)
                .cross(&(tension(t) * (p5 - p4).normalize()))
                .dot(&axis)
        };
        Sample {
            deg,
            t_all: tension(t_all),
            t_isp: tension(t_isp),
            len_all: data.ten_length[t_all],
            len_isp: data.ten_length[t_isp],
            m_qfrc: data.qfrc_spring[0],
            m_oracle: moment("l4_all", "l5_all", t_all) + moment("l4_isp", "l5_isp", t_isp),
        }
    };

    println!(
        "\n{:>7} {:>10} {:>10} {:>12} {:>12} {:>10}",
        "θ(deg)", "T_ALL(N)", "T_ISP(N)", "M_qfrc", "M_oracle", "|Δ|"
    );
    let samples: Vec<Sample> = [0.0_f64, -12.0, -8.0, -4.0, 4.0, 8.0, 12.0]
        .iter()
        .map(|&d| {
            let s = measure(d);
            println!(
                "{:>7.1} {:>10.3} {:>10.3} {:>12.3} {:>12.3} {:>10.2e}",
                s.deg,
                s.t_all,
                s.t_isp,
                s.m_qfrc,
                s.m_oracle,
                (s.m_qfrc - s.m_oracle).abs()
            );
            s
        })
        .collect();

    // (1) Per-angle laws: tension = K·(L−slack) pull-only; engine moment = oracle;
    //     and (for displaced angles) the moment is RESTORING (opposes θ).
    for s in &samples {
        for (tension, length, slack_len) in [
            (s.t_all, s.len_all, slack_all),
            (s.t_isp, s.len_isp, slack_isp),
        ] {
            let predicted = (K * (length - slack_len)).max(0.0);
            assert!(
                (predicted - tension).abs() < 1e-6 * K.max(1.0),
                "tension must equal K·(L−slack): predicted {predicted:.4} vs actual {tension:.4}"
            );
        }
        assert!(
            (s.m_qfrc - s.m_oracle).abs() < 1e-3 * s.m_oracle.abs().max(1.0),
            "engine ligament moment {:.4} must match hand oracle {:.4}",
            s.m_qfrc,
            s.m_oracle
        );
        if s.deg != 0.0 {
            assert!(
                s.m_qfrc * s.deg < 0.0,
                "ligament moment must oppose θ={} (restoring), got {}",
                s.deg,
                s.m_qfrc
            );
        }
    }

    // (2) Neutral pose is force-free (both ligaments exactly at slack).
    let neutral = samples.iter().find(|s| s.deg == 0.0).unwrap();
    assert!(
        neutral.t_all < 1e-6 && neutral.t_isp < 1e-6 && neutral.m_qfrc.abs() < 1e-6,
        "neutral pose must be slack: T_ALL={} T_ISP={} M={}",
        neutral.t_all,
        neutral.t_isp,
        neutral.m_qfrc
    );

    // (3) Bidirectional pull-only: at every displaced angle exactly one ligament
    //     is taut and the other exactly slack; the taut one is consistent within
    //     a direction and SWAPS between flexion and extension.
    let taut = |s: &Sample| -> &'static str {
        let all_only = s.t_all > 1e-3 && s.t_isp < 1e-6;
        let isp_only = s.t_isp > 1e-3 && s.t_all < 1e-6;
        assert!(
            all_only ^ isp_only,
            "exactly one ligament must be taut at θ={} (pull-only)",
            s.deg
        );
        if all_only { "ALL" } else { "ISP" }
    };
    let taut_for = |sign: f64| -> &'static str {
        let which: Vec<_> = samples
            .iter()
            .filter(|s| s.deg * sign > 0.0)
            .map(taut)
            .collect();
        assert!(
            which.iter().all(|&t| t == which[0]),
            "the same ligament must carry all load on one side of neutral"
        );
        which[0]
    };
    assert!(
        taut_for(1.0) != taut_for(-1.0),
        "the taut ligament must swap between flexion and extension"
    );

    println!(
        "\n[rung5] PASS — field-derived ligament sites; pull-only in both directions; \
         restoring moment matches the hand oracle (Σ r×F·axis)."
    );
}

/// One static evaluation of the segment at a given flexion angle (degrees).
struct Sample {
    deg: f64,
    t_all: f64,
    t_isp: f64,
    len_all: f64,
    len_isp: f64,
    m_qfrc: f64,
    m_oracle: f64,
}
