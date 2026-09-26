//! K6's fixture (plan §16b): the block, the cylinder's grid, and the stick
//! zone's readout, which is settled here against the closed forms before K6
//! runs.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::Obstacle;
use sim_soft_explicit::f64::{Material, pose_to_body};
use sim_soft_explicit::fixtures::partial_slip::{
    Block, Cylinder, Leg, PartialSlipRun, STICKING_DEFICIT, Zone, contact_zone,
    stick_while_loading, stick_while_unloading, stick_zone,
};

const A: f64 = 1.0e-3;
const FRICTION: f64 = 0.3;

/// The plan's readout bar: 0.005a (§16b).
const RESOLUTION: f64 = 0.005;

/// Hertz's plane pressure of half-width `r`, over its peak at half-width `a`:
/// `(r/a) √(1 − x²/r²)` inside, 0 outside.
fn hertz(x: f64, r: f64) -> f64 {
    if x.abs() < r {
        r / A * (1.0 - (x / r).powi(2)).sqrt()
    } else {
        0.0
    }
}

/// The closed forms, sampled at nodes `h` apart from `offset`: each node's
/// normal force and its friction along the leg's direction, in units of the
/// peak pressure. Loading to `Q/(μ_f P)` (Cattaneo–Mindlin), or, with
/// `unloading`, falling back by `ΔQ/(μ_f P)` from a peak of 0.8
/// (Mindlin–Deresiewicz), with the friction read along the reverse.
fn sampled(h: f64, offset: f64, fraction: f64, unloading: bool) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let positions: Vec<f64> = (-100..=100).map(|i| offset + f64::from(i) * h).collect();
    let normal = positions.iter().map(|&x| hertz(x, A)).collect();
    let tangential = positions
        .iter()
        .map(|&x| {
            if unloading {
                let peak_stick = A * (1.0_f64 - 0.8).sqrt();
                let m = A * stick_while_unloading(fraction);
                let q = -hertz(x, A) + 2.0 * hertz(x, m) - hertz(x, peak_stick);
                -FRICTION * q
            } else {
                let c = A * stick_while_loading(fraction);
                FRICTION * (hertz(x, A) - hertz(x, c))
            }
        })
        .collect();
    (positions, normal, tangential)
}

/// The largest error of a stick-zone rule over mesh offsets and load
/// fractions, over `a`, for loading and for unloading; and of the contact
/// zone.
fn resolution(
    divisions: f64,
    rule: impl Fn(&[f64], &[f64], &[f64]) -> Option<Zone>,
) -> (f64, f64, f64) {
    let h = A / divisions;
    let (mut contact, mut loading, mut unloading) = (0.0_f64, 0.0_f64, 0.0_f64);
    for step in 0..25 {
        let offset = h * f64::from(step) / 25.0;
        for k in 0..=24 {
            let fraction = 0.2 + 0.6 * f64::from(k) / 24.0;
            let (x, normal, tangential) = sampled(h, offset, fraction, false);
            let zone = contact_zone(&x, &normal).unwrap();
            contact = contact.max((zone.half_width() - A).abs() / A);
            let c = A * stick_while_loading(fraction);
            let read = rule(&x, &normal, &tangential).map_or(0.0, |z| z.half_width());
            loading = loading.max((read - c).abs() / A);
            let (x, normal, tangential) = sampled(h, offset, fraction, true);
            let m = A * stick_while_unloading(fraction);
            let read = rule(&x, &normal, &tangential).map_or(0.0, |z| z.half_width());
            unloading = unloading.max((read - m).abs() / A);
        }
    }
    (contact, loading, unloading)
}

/// The rule the plan first proposed: `(1 − f_t/(μ_f f_n))²` extrapolated
/// linearly to zero from the last two sticking nodes.
fn ratio_rule(x: &[f64], normal: &[f64], tangential: &[f64]) -> Option<Zone> {
    let ratio: Vec<f64> = normal
        .iter()
        .zip(tangential)
        .map(|(&n, &t)| if n > 0.0 { t / (FRICTION * n) } else { 1.0 })
        .collect();
    let sticking: Vec<usize> = (0..x.len()).filter(|&i| ratio[i] < 1.0 - 1e-9).collect();
    let (&first, &last) = (sticking.first()?, sticking.last()?);
    let edge = |outer: usize, inner: usize| {
        let (g1, g2) = ((1.0 - ratio[outer]).powi(2), (1.0 - ratio[inner]).powi(2));
        x[outer] + g1 * (x[outer] - x[inner]) / (g2 - g1)
    };
    Some(Zone {
        left: edge(first, first + 1),
        right: edge(last, last - 1),
    })
}

#[test]
fn the_readout_resolves_the_closed_forms_within_the_plans_bar() {
    let deficit = |x: &[f64], n: &[f64], t: &[f64]| stick_zone(x, n, t, FRICTION);
    let (contact, loading, unloading) = resolution(50.0, deficit);
    eprintln!(
        "MARGIN K6 readout at a/h 50: contact {contact:.5}a, stick loading {loading:.5}a, \
         unloading {unloading:.5}a (bar {RESOLUTION}a)"
    );
    assert!(contact <= RESOLUTION, "contact zone off by {contact}a");
    assert!(
        loading <= RESOLUTION,
        "stick zone off by {loading}a while loading"
    );
    assert!(
        unloading <= RESOLUTION,
        "stick zone off by {unloading}a while unloading"
    );

    // The plan's first candidate misses the bar: dividing by the pressure
    // brings in the pressure's own square root at the contact's edge.
    let (_, ratio_loading, ratio_unloading) = resolution(50.0, ratio_rule);
    eprintln!(
        "MARGIN the ratio rule at a/h 50: loading {ratio_loading:.5}a, unloading \
         {ratio_unloading:.5}a"
    );
    assert!(ratio_loading.max(ratio_unloading) > RESOLUTION);
}

#[test]
fn the_readout_at_the_coarser_meshes_is_recorded() {
    let deficit = |x: &[f64], n: &[f64], t: &[f64]| stick_zone(x, n, t, FRICTION);
    for divisions in [25.0, 12.0] {
        let (contact, loading, unloading) = resolution(divisions, deficit);
        eprintln!(
            "MARGIN K6 readout at a/h {divisions}: contact {contact:.5}a, stick loading \
             {loading:.5}a, unloading {unloading:.5}a"
        );
        // CI's coarse K6 has a tolerance of 0.1 (plan §16b).
        assert!(contact.max(loading).max(unloading) < 0.1 / 4.0);
    }
}

#[test]
fn a_slipping_node_that_stuck_on_some_steps_still_reads_as_slipping() {
    let h = A / 50.0;
    let (x, normal, tangential) = sampled(h, 0.3 * h, 0.5, false);
    let exact = stick_zone(&x, &normal, &tangential, FRICTION).unwrap();
    // Every slipping node's friction 1 % short of its limit.
    let short: Vec<f64> = normal
        .iter()
        .zip(&tangential)
        .map(|(&n, &t)| {
            let limit = FRICTION * n;
            if (limit - t).abs() <= 1e-12 * limit.max(1e-300) && n > 0.0 {
                0.99 * limit
            } else {
                t
            }
        })
        .collect();
    let read = stick_zone(&x, &normal, &short, FRICTION).unwrap();
    const { assert!(STICKING_DEFICIT > 0.01) };
    assert_eq!(read, exact, "a 1 % deficit moved the stick zone");
}

/// A row of eleven nodes 1 apart, all in contact at a limit of 1
/// (`μ_f f_n`), with the deficits given and the rest slipping.
fn stick_row(deficits: &[(usize, f64)]) -> Option<Zone> {
    let x: Vec<f64> = (0..11).map(f64::from).collect();
    let normal = vec![1.0 / FRICTION; 11];
    let mut tangential = vec![1.0; 11];
    for &(i, d) in deficits {
        tangential[i] = 1.0 - d;
    }
    stick_zone(&x, &normal, &tangential, FRICTION)
}

#[test]
fn an_edge_reaches_no_further_than_the_next_node() {
    // The deficit barely rises inwards from the zone's last nodes, so the
    // extrapolation reaches almost nine spacings out; runs' edges often do
    // (plan §16q). It stops at the first slipping node.
    let zone = stick_row(&[(3, 0.9), (4, 0.95), (5, 1.0), (6, 0.95), (7, 0.9)]).unwrap();
    assert_eq!(
        zone,
        Zone {
            left: 2.0,
            right: 8.0
        }
    );
}

#[test]
fn a_sticking_node_apart_from_the_stick_zone_is_not_part_of_it() {
    // A lone node at the contact's edge, two slipping nodes from the zone, as
    // runs have (plan §16q). The zone is the run around the largest deficit.
    let zone = stick_row(&[(4, 0.3), (5, 1.0), (6, 0.3), (9, 0.2)]).unwrap();
    let alone = stick_row(&[(4, 0.3), (5, 1.0), (6, 0.3)]).unwrap();
    assert_eq!(zone, alone);
    assert!(zone.right < 7.0);
}

#[test]
fn an_edge_whose_square_falls_inwards_stays_on_its_last_node() {
    let zone = stick_row(&[(2, 0.6), (3, 0.5), (4, 1.0), (5, 0.5), (6, 0.6)]).unwrap();
    assert_eq!(
        zone,
        Zone {
            left: 2.0,
            right: 6.0
        }
    );
}

#[test]
fn the_closed_forms_are_the_published_ones() {
    assert_eq!(stick_while_loading(0.0), 1.0);
    assert!((stick_while_loading(0.8) - 0.2_f64.sqrt()).abs() < 1e-15);
    assert_eq!(stick_while_loading(1.0), 0.0);
    assert!((stick_while_unloading(0.8) - 0.6_f64.sqrt()).abs() < 1e-15);
    assert_eq!(stick_while_unloading(2.0), 0.0);
}

const SILICONE: Material = Material {
    mu: 23.0e3,
    lambda: 23.0e3 * 2.0 * 0.49 / (1.0 - 2.0 * 0.49),
    c2: 0.0,
    viscosity: 0.0,
    density: 1070.0,
};

#[test]
fn the_block_is_the_plans() {
    let block = Block::plan(A, 50.0);
    let (columns, layers) = (block.columns(), block.layers());
    let h = A / 50.0;
    // 20a wide and 10a deep, symmetric, with a/50 over |x| ≤ 1.5a and the
    // top 0.5a, graded to about a/2.
    assert_eq!(columns[0], -10.0 * A);
    assert_eq!(columns[columns.len() - 1], 10.0 * A);
    assert!(
        columns
            .iter()
            .zip(columns.iter().rev())
            .all(|(l, r)| (l + r).abs() < 1e-15)
    );
    assert_eq!(layers[0], 0.0);
    assert_eq!(layers[layers.len() - 1], -10.0 * A);
    let fine = |p: &[f64], from: f64, to: f64| {
        p.windows(2)
            .filter(|w| w[0] >= from - 1e-12 && w[1] <= to + 1e-12)
            .all(|w| ((w[1] - w[0]) - h).abs() < 1e-12)
    };
    assert!(fine(&columns, -1.5 * A, 1.5 * A));
    let depths: Vec<f64> = layers.iter().map(|z| -z).collect();
    assert!(fine(&depths, 0.0, 0.5 * A));
    let grown = |p: &[f64]| {
        let last = p[p.len() - 1] - p[p.len() - 2];
        (0.45..=0.55).contains(&(last / A))
    };
    assert!(grown(&columns));
    assert!(grown(&depths));
    let cells = |p: &[f64]| p.windows(2).map(|w| w[1] - w[0]).collect::<Vec<f64>>();
    assert!(cells(&columns).iter().all(|&c| c > 0.0));
    assert!(cells(&depths).iter().all(|&c| c > 0.0));

    let model = block.model(SILICONE).unwrap();
    assert_eq!(model.node_count(), columns.len() * 2 * layers.len());
    assert_eq!(
        model.element_count(),
        6 * (columns.len() - 1) * (layers.len() - 1)
    );
    assert!(model.rest_volumes().iter().all(|&v| v > 0.0));
    let volume: f64 = model.rest_volumes().iter().sum();
    assert!((volume / (20.0 * A * 10.0 * A * h) - 1.0).abs() < 1e-9);
    for (p, &held) in model.rest_positions().iter().zip(model.held()) {
        assert_eq!(held, p[2] == -10.0 * A);
    }
    assert!(
        model
            .constraints()
            .iter()
            .all(|c| *c == [[0.0, 1.0, 0.0], [0.0; 3]])
    );
    let row = block.top_row(1);
    assert_eq!(row.len(), columns.len());
    for (&n, &x) in row.iter().zip(&columns) {
        assert_eq!(model.rest_positions()[n as usize], [x, h, 0.0]);
    }
}

/// The cylinder's obstacle at a pose, sampled at a world point.
fn reading(obstacle: &Obstacle, cylinder: &Cylinder, x: f64, z: f64, point: [f64; 3]) -> f64 {
    obstacle
        .sample(pose_to_body(cylinder.pose(x, z), point))
        .distance
}

#[test]
fn the_baked_cylinder_is_exact_at_the_contact_and_outside_everywhere_else() {
    // The plan's turn, and others: turned ±10°, the box that just holds the
    // strip has a corner 0.04a inside the cylinder that the top's far nodes
    // are clamped onto, one side or the other; and 60°.
    for (divisions, radius, turn) in [
        (50.0, 100.0, 30.0),
        (12.0, 200.0, 30.0),
        (12.0, 100.0, 10.0),
        (12.0, 100.0, -10.0),
        (12.0, 100.0, 60.0),
    ] {
        let block = Block::plan(A, divisions);
        let cylinder = Cylinder {
            radius: radius * A,
            turn: f64::to_radians(turn),
        };
        let (grid, values) = cylinder.baked(&block, block.fine / 2.0).unwrap();
        let obstacle = Obstacle {
            grid,
            values,
            start: 0.0,
            interval: 1.0,
            poses: vec![cylinder.pose(0.0, 0.0)],
            friction: FRICTION,
        };
        let model = block.model(SILICONE).unwrap();
        // Pressed deeper than K6 goes, and pushed either way further.
        let mut worst = 0.0_f64;
        for (x, z) in [(0.0, 0.0), (-0.05 * A, -0.04 * A), (0.05 * A, -0.04 * A)] {
            for p in model.rest_positions() {
                let exact = cylinder.distance(pose_to_body(cylinder.pose(x, z), *p));
                let read = reading(&obstacle, &cylinder, x, z, *p);
                assert_eq!(
                    read <= 0.0,
                    exact <= 0.0,
                    "a/h {divisions}, R {radius}a, turn {turn}°: node {p:?} reads {read:e}, \
                     exact {exact:e}"
                );
                // Where a node can touch within a step or so.
                if exact < 0.01 * A {
                    worst = worst.max((read - exact).abs());
                }
            }
        }
        eprintln!(
            "MARGIN baked cylinder, a/h {divisions}, R {radius}a, turn {turn}°: {:.2e}a within \
             0.01a of it",
            worst / A
        );
        assert!(worst < 1e-6 * A);
    }
}

#[test]
fn a_coarse_k6_runs_from_the_press_to_the_return() {
    // K6's whole path at a/h 4, four times faster than the plan's loading: a
    // sanity run, as `tube_sanity` is the tube's, with no accuracy claim.
    let run = PartialSlipRun::plan(4.0).slowed(0.25);
    let result = run
        .run(|model, obstacle| cpu::f64::CpuExecutor::new(model, obstacle).unwrap())
        .unwrap();
    let judged = |leg: Leg| {
        result
            .readings
            .iter()
            .filter(|r| r.leg == leg)
            .filter(|r| result.fraction(r).is_some_and(|f| (0.2..=0.8).contains(&f)))
            .count()
    };
    let errors = result.errors(0.2, 0.8);
    eprintln!(
        "MARGIN K6 sanity at a/h 4: {} steps, errors {errors:?}, energy balance {:?}",
        result.steps, result.energy_balance
    );
    assert!(judged(Leg::Push) > 0 && judged(Leg::Return) > 0);
    assert!(errors.worst().is_some_and(f64::is_finite));
    assert!(result.energy_balance.is_some_and(|e| e <= 0.01));
    assert!(!result.inverted);
}
