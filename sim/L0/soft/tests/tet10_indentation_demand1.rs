//! Tet10 ladder rung 6d — **demand #1**, the curved-contact-patch over-stiffness
//! floor, measured on the net-force metric.
//!
//! # Result — Tet10 improves demand #1 but does not close it
//!
//! Measured 2026-07-25, both arms from **one release build**. The scalars are
//! pinned as committed constants below and re-derived by running the two
//! `probe_*` tests; the differential node-density control is executable in
//! [`node_density_control_confound_is_small`].
//!
//! | element | `RATIO` (χ = 0.50) | over-stiffness | mean `sd/δ` | nodes |
//! |---|---|---|---|---|
//! | Tet4 | 1.1303 | +13.0 % | 0.04213 | 4 375 |
//! | Tet10 | 1.0803 | +8.0 % | 0.04607 | 31 213 |
//!
//! The Tet4 arm reproduces `#676`'s committed `RATIO = 1.130` to its three
//! recorded decimals (harness validation — same binary as the Tet10 arm). The Tet10 element pulls
//! the χ = 0.50 over-stiffness from **+13.0 % down to +8.0 %** — a real ≈5-point
//! improvement, of which the differential contact-discretisation confound
//! accounts for at most ~0.6 points (see the control section), so it is
//! predominantly element order. But a **+8 % residual remains**: the element
//! upgrade alone does **not** close demand #1.
//!
//! **What this rung does and does not establish.** It establishes that a
//! higher-order element materially reduces the curved-patch over-stiffness at
//! this χ. It does **not** attribute the +8 % residual — a single-χ net-force
//! measurement cannot separate the still-inconsistent node-collocated contact
//! patch (a known un-fixed contributor, ladder **rung 8**'s remit) from oracle
//! (Garcia-correction) error, residual element stiffness, or incomplete mesh
//! convergence. And note the basis: `#676`'s headline "~9 %" is Tet4's
//! *mesh-converged* floor (extrapolated over three χ); the numbers here are a
//! *single* χ (= 0.50), where Tet4 itself reads +13 %, not +9 %. A Tet10
//! mesh-converged floor would need its own three-χ sweep at ~3× the 60-min
//! cost and is **not** measured. So: demand #1 is *improved*, not settled —
//! and the residual is not yet attributed.
//!
//! **Cost reality, measured:** `#676`'s Tet4 3-χ sweep is **24.2 min**; the
//! single-χ Tet4 arm here is **61 s**; the Tet10 arm at **~7× the DOF**
//! (93 639 vs 13 125) ran **59.6 min** single-threaded — 100 % of one core, the
//! assembly being serial by design for bit-determinism. The Tet10 arm is
//! therefore a **deliberately-run one-shot measurement**, `#[ignore]`d
//! unconditionally: never a routine or CI test. What CI *does* run is the cheap
//! [`node_density_control_confound_is_small`] relationship guard over the pinned
//! constants. See the session memory `project-tet10-fbar-element-upgrade` for
//! the full carry-forward.
//!
//! ## What this rung answers
//!
//! Plan §1 names two demands. Rung 6c settled demand #2 (near-incompressible
//! accuracy at ν = 0.49 → ACCEPT). Demand #1 is the other one: `#676`
//! (`bonded_layer_indentation.rs`) proved with a 3-level convergence study that
//! Tet4 over-stiffens the curved contact patch by a **mesh-converged ~9 %** —
//! measured `RATIO = F_FEM / (Δ_G·Hertz)` of 1.051 / 1.105 / 1.130 across
//! χ ∈ {0.20, 0.35, 0.50}, inside a documented `[1.00, 1.20]` band. That gate's
//! own docstring attributes the floor to "Tet4 element over-stiffness on the
//! curved contact patch" and names Tet10 as the fix. This file tests that.
//!
//! **Scope, stated up front: this is the NET-FORCE metric only.** Contact here
//! is a blind per-vertex loop against an analytic rigid SDF, so Tet10's midside
//! nodes automatically carry barrier force with zero contact-code change — but
//! node-collocated barriers on a quadratic face are *inconsistent* (under
//! uniform pressure a quadratic triangle loads midsides, not corners), so the
//! **local** pressure/patch mechanics stay wrong until ladder rung 8's
//! surface-integrated barrier. `F_FEM` is a robust force sum and is unaffected;
//! `peak_contact_pressure` is not, and is not read here.
//!
//! ## ★ The node-density control, and why it is mandatory
//!
//! The IPC barrier is **fixed-κ and not area-weighted** (`contact/ipc.rs`), so
//! its effective compliance scales with the number of surface nodes in the
//! band. Tet10 roughly doubles them. Swap the element and `RATIO` moves for
//! **two** reasons — element order *and* contact discretisation — and the gate
//! cannot separate them. Plan §5 step 6d therefore requires a control: either
//! run a Tet4 mesh refined to Tet10's surface-node count, or verify the barrier
//! is near-rigid so `δ_eff ≈ δ` independent of node count.
//!
//! **This file takes the second route, as a measurement rather than an
//! argument.** [`ContactPairReadout::sd`] is the signed standoff of each active
//! contact vertex, and that standoff *is* the error in effective indentation:
//! a vertex held off the indenter by `sd` is one whose material never saw that
//! part of the indentation, so `δ_eff ≈ δ − sd`. With `F ∝ δ^{3/2}` the induced
//! force error is `≈ 1.5·sd/δ`.
//!
//! **The absolute standoff is common to both elements and cancels; only the
//! *differential* biases the RATIO comparison.** Both elements sit at a mean
//! `sd/δ ≈ 0.042–0.046` (~84–92 % of the `d̂/δ = 0.05` band) — so the barrier is
//! *not* near-rigid at the shipped κ, but that compliance is shared. The
//! confound is the difference: `1.5·|0.04607 − 0.04213| = 0.59 %`, against a
//! **5.0-point** (Tet4 − Tet10 `RATIO`) element-order signal — ~12 % of it. Real
//! but not dominant: it cannot flip the finding, so the refined-Tet4 fallback
//! (`contact_stability.rs` shows κ has a convergence ceiling, so raising it is
//! not free) is **not** triggered. This relationship is asserted over the
//! committed constants in [`node_density_control_confound_is_small`].
//!
//! ## Why one χ, and which
//!
//! `#676` sweeps three χ over a 71-increment displacement ramp — 213 solves,
//! and it is already the slow route (minutes in release; now gated by the
//! path-filtered `soft-contact-heavy` CI job, #692). Tet10 at
//! ~7× the DOF (measured) is not viable across the full sweep. This file runs
//! **χ = 0.50**, which is both the cheapest mesh (20,736 tets against 55,296 at
//! χ = 0.20) *and* the strongest signal — `#676` measures Tet4's largest
//! over-stiffness there (`RATIO` 1.130). Cheapest and most discriminating is a
//! happy coincidence, not a compromise.
//!
//! The Tet4 arm is re-solved **live** rather than read from `#676`'s committed
//! band, so both elements come from one build of one harness at one χ.

// Saint-Venant-style sums over active pairs and the cell-count arithmetic cast
// small counts to `f64` — the same idiom `bonded_layer_indentation.rs` uses.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

mod common;

use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::material::NeoHookean;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, IpcRigidContact, MaterialField, Mesh,
    Solver, SolverConfig, SphereSdf, Tet4, Tet10Mesh, TranslatedSdf, Vec3,
    pick_vertices_by_predicate,
};

// ── Scene constants — mirroring `bonded_layer_indentation.rs` exactly ─────

const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0 * MU;
const RADIUS: f64 = 1.0e-2;
const DELTA: f64 = 5.0e-4;
const KAPPA: f64 = 1.0e4;
const BAND_FRAC: f64 = 0.05;
const STATIC_DT: f64 = 1.0;
const LATERAL_FACTOR: f64 = 8.0;
const A_OVER_CELL: f64 = 3.0;

/// The single swept χ — cheapest mesh and largest Tet4 over-stiffness (see
/// module docs). `#676` measures `RATIO = 1.130` here.
const CHI: f64 = 0.50;

/// Newton cap per ramp increment. `#676` uses 80 for Tet4. Rung 6c measured
/// near-incompressible Tet10 needing 272 iterations in bending, so this is
/// raised deliberately rather than inherited — but ν = 0.4 here, well away from
/// that regime, and [`MAX_EXPECTED_ITERS`] is the bound that discriminates.
const MAX_NEWTON_ITER: usize = 400;

/// Healthy-convergence bound on Newton iterations per increment, asserted by
/// both probes. Measured maxima at χ = 0.50, ν = 0.4: **Tet4 = 8, Tet10 = 10**.
/// A solve needing many more is stalling (the near-incompressible 272-iteration
/// regime rung 6c hit) — this bound, not the [`MAX_NEWTON_ITER`] solver cap
/// (400), discriminates a healthy measurement from a degenerate one. The 4×
/// headroom over the measured maximum absorbs active-set churn without masking a
/// real convergence regression.
const MAX_EXPECTED_ITERS: usize = 40;

// ── Committed measurements (rung 6d, χ = 0.50; one release build, 2026-07-25) ─
//
// Both arms come from a single binary. The Tet4 arm reproduces `#676`'s
// committed RATIO = 1.130 to its three recorded decimals (harness validation); see the module
// header for the full table and what the numbers do and do not establish. The
// two heavy `probe_*` tests re-derive these live when deliberately run; the
// cheap `node_density_control_confound_is_small` test enforces the differential
// control over them in CI without solving.

/// Tet4 `RATIO` at χ = 0.50 — equals `#676`'s committed value (harness check).
const TET4_RATIO: f64 = 1.1303;
/// Tet10 `RATIO` at χ = 0.50 — the demand-#1 measurement.
const TET10_RATIO: f64 = 1.0803;
/// Tet4 mean standoff `sd/δ` over active pairs at the final pose.
const TET4_MEAN_SD_OVER_DELTA: f64 = 0.04213;
/// Tet10 mean standoff `sd/δ` over active pairs at the final pose.
const TET10_MEAN_SD_OVER_DELTA: f64 = 0.04607;

/// Relative regression band on the pinned scalars. The probes never run in CI
/// (both `#[ignore]`d in debug; not in any release `--test` list), so this only
/// bites a deliberate re-run; 2 % catches a real element/contact regression —
/// the Tet4↔Tet10 `RATIO` gap alone is 4.4 % of the value — while absorbing
/// cross-platform faer drift over 71 warm-started nonlinear increments. The same
/// band covers the standoff means (2 % of ~0.046 ≈ 9e-4, orders above solver
/// drift on a geometric quantity read from converged positions).
const COMMITTED_REL: f64 = 0.02;

// ── Analytic oracle — the bonded bottom-effect correction to Hertz ────────

fn nu() -> f64 {
    LAMBDA / (2.0 * (LAMBDA + MU))
}

fn young() -> f64 {
    MU * (3.0 * LAMBDA + 2.0 * MU) / (LAMBDA + MU)
}

fn e_star() -> f64 {
    young() / (1.0 - nu() * nu())
}

fn hertz_halfspace(delta: f64) -> f64 {
    4.0 / 3.0 * e_star() * RADIUS.sqrt() * delta.powf(1.5)
}

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

fn layer(nx: usize, ny: usize, nz: usize, lx: f64, ly: f64, h: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, lx, ly, h, &material_field())
}

fn indenter(lx: f64, ly: f64, z_center: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: RADIUS },
        offset: Vec3::new(lx / 2.0, ly / 2.0, z_center),
    }
}

/// `(n_lat, nz, lateral, h)` for `CHI` — identical arithmetic to `#676`.
fn dims_for(chi: f64) -> (usize, usize, f64, f64) {
    let a = (RADIUS * DELTA).sqrt();
    let cell = a / A_OVER_CELL;
    let h = a / chi;
    let lateral = LATERAL_FACTOR * a;
    let even = |n: usize| if n.is_multiple_of(2) { n } else { n + 1 };
    let n_lat = even(((lateral / cell).round() as usize).max(2));
    let nz = even(((h / cell).round() as usize).max(2));
    (n_lat, nz, lateral, h)
}

// ── Harness ──────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ElementOrder {
    Tet4,
    Tet10,
}

/// One indentation run plus the contact-discretisation diagnostics the
/// node-density control needs.
struct Indentation {
    /// `F_FEM / (Δ_G·Hertz)` — the demand-#1 metric.
    ratio: f64,
    f_fem: f64,
    n_active: usize,
    n_nodes: usize,
    max_iters: usize,
    /// Largest standoff over active pairs, in units of `δ`. This is the bound
    /// on the effective-indentation error (module docs).
    max_sd_over_delta: f64,
    /// Mean standoff over active pairs, in units of `δ`.
    mean_sd_over_delta: f64,
}

// The ramp body is one sequential procedure — mesh + BC setup, the increment
// loop, then the readout — and `#676`'s equivalent is written the same way.
// Splitting it would thread the pose, the warm-started state and the two
// element branches through helper signatures for no readability gain.
#[allow(clippy::too_many_lines)]
/// Displacement-controlled IPC indentation at `CHI`, for either element.
/// Mirrors `#676`'s ramp exactly: south pole starts `1.2·d̂` clear of the top
/// face and descends in `0.3·d̂` increments to `δ` below it, warm-started.
fn run_indentation(order: ElementOrder) -> Indentation {
    let (n_lat, nz, lateral, h) = dims_for(CHI);
    let d_hat = BAND_FRAC * DELTA;
    let z_start = h + RADIUS + 1.2 * d_hat;
    let z_end = h + RADIUS - DELTA;
    let step = 0.3 * d_hat;

    let tet4_mesh = layer(n_lat, n_lat, nz, lateral, lateral, h);
    let tet10_mesh = Tet10Mesh::from_tet4(&tet4_mesh);
    let n_nodes = match order {
        ElementOrder::Tet4 => tet4_mesh.n_vertices(),
        ElementOrder::Tet10 => tet10_mesh.n_vertices(),
    };
    let n_dof = 3 * n_nodes;

    // The bonded base: every node on `z = 0`. On the enriched mesh a midside
    // between two `z = 0` corners lies exactly on the plane and is pinned;
    // one on an edge rising into the layer is not. No Tet10-specific logic.
    let pins = match order {
        ElementOrder::Tet4 => pick_vertices_by_predicate(&tet4_mesh, |p| p.z.abs() < 1e-9),
        ElementOrder::Tet10 => pick_vertices_by_predicate(&tet10_mesh, |p| p.z.abs() < 1e-9),
    };
    assert!(!pins.is_empty(), "bonded bottom face has no nodes at z = 0");

    let rest: Vec<Vec3> = match order {
        ElementOrder::Tet4 => tet4_mesh.positions().to_vec(),
        ElementOrder::Tet10 => tet10_mesh.positions().to_vec(),
    };
    let mut x_prev: Vec<f64> = rest.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    let v_prev = vec![0.0_f64; n_dof];

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    let empty_theta = Tensor::from_slice(&[], &[0]);

    let n_increments = ((z_start - z_end) / step).ceil() as usize;
    eprintln!(
        "rung-6d {order:?} @ χ={CHI}: starting {n_increments} increments over {n_nodes} nodes \
         ({n_dof} DOF)"
    );
    let mut z = z_start;
    let mut max_iters = 0usize;
    let mut increment = 0usize;
    loop {
        let target = if z - step <= z_end { z_end } else { z - step };
        z = target;
        increment += 1;
        let contact =
            IpcRigidContact::with_params(vec![indenter(lateral, lateral, z)], KAPPA, d_hat);
        let bc = BoundaryConditions::new(pins.clone(), Vec::new());
        let out = match order {
            ElementOrder::Tet4 => {
                let solver: CpuNewtonSolver<Tet4, HandBuiltTetMesh, IpcRigidContact> =
                    CpuNewtonSolver::new(
                        Tet4,
                        layer(n_lat, n_lat, nz, lateral, lateral, h),
                        contact,
                        cfg,
                        bc,
                    );
                solver.replay_step(
                    &Tensor::from_slice(&x_prev, &[n_dof]),
                    &Tensor::from_slice(&v_prev, &[n_dof]),
                    &empty_theta,
                    STATIC_DT,
                )
            }
            ElementOrder::Tet10 => {
                let solver: CpuNewtonSolver<Tet10, Tet10Mesh, IpcRigidContact, NeoHookean, 10, 4> =
                    CpuNewtonSolver::new(
                        Tet10,
                        Tet10Mesh::from_tet4(&layer(n_lat, n_lat, nz, lateral, lateral, h)),
                        contact,
                        cfg,
                        bc,
                    );
                solver.replay_step(
                    &Tensor::from_slice(&x_prev, &[n_dof]),
                    &Tensor::from_slice(&v_prev, &[n_dof]),
                    &empty_theta,
                    STATIC_DT,
                )
            }
        };
        max_iters = max_iters.max(out.iter_count);
        // Per-increment progress. A 71-increment ramp that prints nothing for
        // tens of minutes is unauditable — you cannot tell a slow solve from a
        // hung one, which is exactly what happened on the first Tet10 run.
        eprintln!(
            "  [{order:?} increment {increment}/{n_increments}: newton iters = {i}, \
             residual = {r:e}]",
            i = out.iter_count,
            r = out.final_residual_norm,
        );
        x_prev = out.x_final;
        if (z - z_end).abs() < 1e-12 {
            break;
        }
    }

    // Reaction force + standoff distribution at the final pose.
    let readout_contact =
        IpcRigidContact::with_params(vec![indenter(lateral, lateral, z_end)], KAPPA, d_hat);
    let positions: Vec<Vec3> = x_prev
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    let readout = match order {
        ElementOrder::Tet4 => readout_contact.per_pair_readout(&tet4_mesh, &positions),
        ElementOrder::Tet10 => readout_contact.per_pair_readout(&tet10_mesh, &positions),
    };
    assert!(!readout.is_empty(), "{order:?}: no active contact pairs");

    let f_fem = readout.iter().map(|r| r.force_on_soft.z).sum::<f64>().abs();
    let max_sd = readout.iter().map(|r| r.sd.abs()).fold(0.0_f64, f64::max);
    let mean_sd = readout.iter().map(|r| r.sd.abs()).sum::<f64>() / readout.len() as f64;
    let oracle = common::garcia_bonded_correction(nu(), CHI) * hertz_halfspace(DELTA);

    Indentation {
        ratio: f_fem / oracle,
        f_fem,
        n_active: readout.len(),
        n_nodes,
        max_iters,
        max_sd_over_delta: max_sd / DELTA,
        mean_sd_over_delta: mean_sd / DELTA,
    }
}

fn report(order: ElementOrder, r: &Indentation) {
    eprintln!(
        "rung-6d {order:?} @ χ={CHI}: RATIO = {ratio:.4}, F_FEM = {f:.5} N, active pairs = {na}, \
         nodes = {nn}, max newton iters = {mi}, standoff sd/δ: max {msd:.5} mean {asd:.5} \
         (⇒ force-error bound ≈ {fe:.2} %)",
        ratio = r.ratio,
        f = r.f_fem,
        na = r.n_active,
        nn = r.n_nodes,
        mi = r.max_iters,
        msd = r.max_sd_over_delta,
        asd = r.mean_sd_over_delta,
        fe = 100.0 * 1.5 * r.mean_sd_over_delta,
    );
}

// ── Committed-value regression helper ────────────────────────────────────

/// Assert a freshly-measured scalar matches its committed value within
/// [`COMMITTED_REL`], naming both in the failure message.
fn assert_committed(actual: f64, committed: f64, what: &str) {
    let rel = (actual - committed).abs() / committed.abs();
    assert!(
        rel <= COMMITTED_REL,
        "{what}: measured {actual:.5} vs committed {committed:.5} \
         (relative {rel:.4} > band {COMMITTED_REL})",
    );
}

// ── The node-density control, executable in CI over the committed constants ──

/// The differential node-density control (module header, "node-density control"
/// section), as an always-run guard over the pinned constants — no solve, so it
/// runs in `tests-debug` where the heavy probes cannot.
///
/// The absolute standoff is common to both elements and cancels; only the
/// *differential* biases the `RATIO` comparison. With `F ∝ δ^{3/2}` the induced
/// force-error bound is `1.5·|Δ(mean sd/δ)|`. Require it small against the
/// element-order signal (the Tet4 − Tet10 `RATIO` gap): if a future re-measure
/// edits a committed mean so the confound rivals the signal, the `RATIO`
/// improvement can no longer be read as element order, and this fails loudly.
/// The 20 % bar is a chosen "cannot-flip-the-finding" margin, not a derived
/// bound — the measured confound is ~12 % of the signal (~1.7× headroom).
#[test]
fn node_density_control_confound_is_small() {
    let signal = TET4_RATIO - TET10_RATIO;
    assert!(
        signal > 0.0,
        "element-order signal must be positive (Tet10 softer than Tet4)"
    );
    let confound = 1.5 * (TET10_MEAN_SD_OVER_DELTA - TET4_MEAN_SD_OVER_DELTA).abs();
    assert!(
        confound < 0.20 * signal,
        "differential contact-discretisation confound {confound:.4} is not small against the \
         {signal:.4} element-order signal (≥20 %) — the RATIO improvement can no longer be \
         attributed to element order; the refined-Tet4 baseline would then be required",
    );
}

// ── Measurement probes — deliberate-run, pin the committed scalars ───────────

/// Tet4 arm, ~61 s in release. Validates the harness against `#676`'s committed
/// `RATIO = 1.130` and pins the Tet4 standoff. `#[ignore]`d in debug only; not
/// in any release `--test` list, so it never runs in CI either.
#[cfg_attr(debug_assertions, ignore = "release-only heavy IPC ramp")]
#[test]
fn probe_tet4_arm_reproduces_676() {
    let r = run_indentation(ElementOrder::Tet4);
    report(ElementOrder::Tet4, &r);
    assert!(
        r.max_iters <= MAX_EXPECTED_ITERS,
        "Tet4 convergence regressed: {} iters > {MAX_EXPECTED_ITERS}",
        r.max_iters,
    );
    assert_committed(r.ratio, TET4_RATIO, "Tet4 RATIO");
    assert_committed(
        r.mean_sd_over_delta,
        TET4_MEAN_SD_OVER_DELTA,
        "Tet4 mean sd/δ",
    );
}

/// Tet10 arm, ~60 min single-threaded (see the module header's cost note).
/// `#[ignore]`d unconditionally: a deliberately-run one-shot measurement, never
/// routine. Pins the Tet10 `RATIO` and standoff.
#[ignore = "rung-6d: ~60 min single-threaded one-shot measurement — run deliberately, never routinely"]
#[test]
fn probe_tet10_arm() {
    let r = run_indentation(ElementOrder::Tet10);
    report(ElementOrder::Tet10, &r);
    assert!(
        r.max_iters <= MAX_EXPECTED_ITERS,
        "Tet10 convergence regressed: {} iters > {MAX_EXPECTED_ITERS}",
        r.max_iters,
    );
    assert_committed(r.ratio, TET10_RATIO, "Tet10 RATIO");
    assert_committed(
        r.mean_sd_over_delta,
        TET10_MEAN_SD_OVER_DELTA,
        "Tet10 mean sd/δ",
    );
}
