//! Tet10 ladder rung 6d — **demand #1**, the curved-contact-patch over-stiffness
//! floor, measured on the net-force metric.
//!
//! # ⚠ WORK IN PROGRESS — not a gate yet
//!
//! Landed state (2026-07-24): the harness is **validated** — its Tet4 arm
//! reproduces `#676`'s committed `RATIO = 1.130` at χ = 0.50 to four decimals.
//! What is NOT done: the Tet10 arm has no committed number (its first run
//! exceeded 40 min and was not allowed to finish), so the node-density control
//! is unresolved and there is no assertion gate. The two `probe_*` tests below
//! are measurement harnesses, not gates. See the session memory
//! `project-tet10-fbar-element-upgrade` for the full carry-forward.
//!
//! **Cost reality, measured:** `#676`'s Tet4 3-χ sweep is **24.2 min**; the
//! single-χ Tet4 arm here is **64 s**; the Tet10 arm at ~2.8× the nodes ran
//! **>40 min** at 1.86 GB without finishing. Whatever 6d becomes, it is not a
//! test anyone runs routinely, and almost certainly not a CI test.
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
//! force error is `≈ 1.5·sd/δ`. So the control is a direct bound, not a
//! plausibility claim: measure the `sd` distribution for **both** elements at
//! the final pose, convert to a force-error bound, and require it to be small
//! against the ~9 % signal being claimed. If the barrier turns out compliant at
//! the shipped κ, this fails loudly and the refined-Tet4 baseline is the
//! fallback — `contact_stability.rs` shows κ has a convergence ceiling, so
//! simply raising it is not free.
//!
//! ## Why one χ, and which
//!
//! `#676` sweeps three χ over a 71-increment displacement ramp — 213 solves,
//! and it is already the slow route (minutes in release, CI-dark). Tet10 at
//! ~2.8× the nodes is not viable across the full sweep. This file runs
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

// ── Measurement probes (NOT gates — see the WIP header) ──────────────────

// 64 s in release. Validates the harness against `#676`'s committed 1.130.
#[cfg_attr(debug_assertions, ignore = "release-only heavy IPC ramp")]
#[test]
fn probe_tet4_arm_reproduces_676() {
    let r = run_indentation(ElementOrder::Tet4);
    report(ElementOrder::Tet4, &r);
}

// ⚠ >40 min and unfinished on first attempt. `#[ignore]` unconditionally so
// it is never picked up by a routine run in either profile.
#[ignore = "rung-6d WIP: exceeded 40 min unfinished — run deliberately, never routinely"]
#[test]
fn probe_tet10_arm() {
    let r = run_indentation(ElementOrder::Tet10);
    report(ElementOrder::Tet10, &r);
}
