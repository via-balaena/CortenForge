//! Fit plan D1's readings (plan §16s): the push over travel, the area
//! percentile, the contact area, and the most-loaded patch, each on a case
//! whose answer is known.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

use std::f64::consts::{FRAC_1_SQRT_2, PI, TAU};

use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::executor::{Obstacle, Snapshot};
use sim_soft_explicit::f64::{Material, Pose, tet4_volume, triangle_area};
use sim_soft_explicit::fixtures::grid::bake;
use sim_soft_explicit::fixtures::tube::{Mesh, Tube, Walls, tributary_area};
use sim_soft_explicit::readings::{PROBE_AREA, WindowContact, area_percentile, travel_peak};

const SILICONE: Material = Material {
    mu: 23.0e3,
    lambda: 23.0e3 * 2.0 * 0.49 / (1.0 - 2.0 * 0.49),
    c2: 0.0,
    viscosity: 0.0,
    density: 1070.0,
};

const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

const MM: f64 = 1e-3;

#[test]
fn the_push_over_travel_is_the_work_over_the_window() {
    // 1 N over the first 5 mm, 3 N over the next 5 and 2 N over the last 10.
    // The best 10 mm window is 5–15 mm: (3 × 5 + 2 × 5) / 10. Its trailing
    // end meets a sample and its leading end does not.
    let samples = [(5.0 * MM, 1.0), (10.0 * MM, 3.0), (20.0 * MM, 2.0)];
    let peak = travel_peak(0.0, &samples, 10.0 * MM).unwrap();
    assert!((peak - 2.5).abs() < 1e-12, "{peak}");
    // A window as long as the path is its mean.
    let whole = travel_peak(0.0, &samples, 20.0 * MM).unwrap();
    assert!((whole - 2.0).abs() < 1e-12, "{whole}");
    // The first interval starts where the travel starts.
    let later = [(15.0 * MM, 1.0), (20.0 * MM, 3.0), (30.0 * MM, 2.0)];
    assert_eq!(travel_peak(10.0 * MM, &later, 10.0 * MM), Some(peak));
    // Here the best window, 2–12 mm, ends on a sample and starts off one:
    // (1 × 8 + 5 × 2) / 10.
    let early = [(10.0 * MM, 1.0), (12.0 * MM, 5.0), (30.0 * MM, 0.0)];
    let peak = travel_peak(0.0, &early, 10.0 * MM).unwrap();
    assert!((peak - 1.8).abs() < 1e-12, "{peak}");
}

#[test]
fn a_ripple_as_long_as_the_node_spacing_averages_out() {
    // A push of 1 N rippling ±1 N with the node spacing's period, read every
    // hundredth of a period over forty periods.
    let spacing = 2.8 * MM;
    let samples: Vec<(f64, f64)> = (1..=4000)
        .map(|i| {
            let x = f64::from(i) * spacing / 100.0;
            let mid = x - 0.5 * spacing / 100.0;
            (x, 1.0 + (TAU * mid / spacing).sin())
        })
        .collect();
    let raw = samples.iter().map(|&(_, f)| f).fold(0.0, f64::max);
    assert!(raw > 1.99, "{raw}");
    // Three whole periods hold no net ripple.
    let peak = travel_peak(0.0, &samples, 3.0 * spacing).unwrap();
    assert!((peak - 1.0).abs() < 1e-9, "{peak}");
}

#[test]
fn a_window_between_whole_periods_keeps_a_share_of_the_ripple() {
    // Over w periods a sinusoidal ripple of amplitude 1 leaves |sin πw| / πw in
    // the windowed mean's peak: 10 mm is 2.92 and 3.58 ring spacings on the
    // 50k and 100k tubes.
    let spacing = 2.8 * MM;
    let samples: Vec<(f64, f64)> = (1..=4000)
        .map(|i| {
            let x = f64::from(i) * spacing / 100.0;
            let mid = x - 0.5 * spacing / 100.0;
            (x, 1.0 + (TAU * mid / spacing).sin())
        })
        .collect();
    for periods in [0.010 / (0.120 / 35.0), 0.010 / (0.120 / 43.0)] {
        let peak = travel_peak(0.0, &samples, periods * spacing).unwrap();
        let kept = (PI * periods).sin().abs() / (PI * periods);
        assert!(
            (peak - 1.0 - kept).abs() < 1e-3,
            "{periods}: {peak} vs {kept}"
        );
    }
}

#[test]
fn a_hold_adds_no_work_and_a_short_or_backward_path_reads_nothing() {
    let mut samples = vec![(5.0 * MM, 1.0), (10.0 * MM, 3.0), (20.0 * MM, 2.0)];
    let before = travel_peak(0.0, &samples, 10.0 * MM);
    samples.extend([(20.0 * MM, 100.0), (20.0 * MM, 100.0)]);
    assert_eq!(travel_peak(0.0, &samples, 10.0 * MM), before);
    assert_eq!(travel_peak(0.0, &samples, 21.0 * MM), None);
    assert_eq!(
        travel_peak(0.0, &[(20.0 * MM, 1.0), (19.0 * MM, 1.0)], 5.0 * MM),
        None
    );
    for window in [0.0, -5.0 * MM, f64::NAN, f64::INFINITY] {
        assert_eq!(travel_peak(0.0, &samples, window), None, "{window}");
    }
}

#[test]
fn the_percentile_is_read_where_the_running_area_reaches_the_fraction() {
    let readings = [(2.0, 1.0), (1.0, 16.0), (5.0, 1.0), (3.0, 1.0), (4.0, 1.0)];
    // 20 in all: the first 1 is 5 % of it, the first 2 is 10 %.
    assert_eq!(area_percentile(&readings, 0.05), Some(5.0));
    assert_eq!(area_percentile(&readings, 0.10), Some(4.0));
    assert_eq!(area_percentile(&readings, 0.11), Some(3.0));
    assert_eq!(area_percentile(&readings, 1.0), Some(1.0));
    assert_eq!(area_percentile(&[], 0.05), None);
    assert_eq!(area_percentile(&[(1.0, 0.0), (2.0, 0.0)], 0.05), None);
}

/// A flat slab of `cells × cells` cubes of side `size`, one cube deep, its
/// top at `z = 0` and centred on the origin, each cube split into six
/// tetrahedra around its diagonal. The top's interior nodes are moved sideways
/// by up to `jitter · size` in a fixed pseudo-random pattern, so its triangles
/// are irregular but flat. Returns the model and its top nodes.
fn slab(cells: usize, size: f64, jitter: f64) -> (ExplicitModel, Vec<u32>) {
    let edge_nodes = cells + 1;
    let node = |column: usize, row: usize, layer: usize| {
        u32::try_from((layer * edge_nodes + row) * edge_nodes + column).unwrap()
    };
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut random = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1_u64 << 53) as f64 - 0.5
    };
    let half = cells as f64 * size / 2.0;
    let mut positions = Vec::new();
    for (layer, height) in [(0, -size), (1, 0.0)] {
        for row in 0..edge_nodes {
            for column in 0..edge_nodes {
                let mut position = [
                    column as f64 * size - half,
                    row as f64 * size - half,
                    height,
                ];
                let interior = (1..cells).contains(&column) && (1..cells).contains(&row);
                if layer == 1 && interior {
                    position[0] += 2.0 * jitter * size * random();
                    position[1] += 2.0 * jitter * size * random();
                }
                positions.push(position);
            }
        }
    }
    let orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut elements = Vec::new();
    for row in 0..cells {
        for column in 0..cells {
            for order in orders {
                let mut corner = [column, row, 0];
                let mut tet = [node(column, row, 0); 4];
                for (slot, axis) in order.into_iter().enumerate() {
                    corner[axis] += 1;
                    tet[slot + 1] = node(corner[0], corner[1], corner[2]);
                }
                let corners: Vec<f64> = tet.iter().flat_map(|&v| positions[v as usize]).collect();
                if tet4_volume(corners.try_into().unwrap()) < 0.0 {
                    tet.swap(1, 2);
                }
                elements.push(tet);
            }
        }
    }
    let count = elements.len();
    let held = (0..positions.len())
        .map(|v| v < edge_nodes * edge_nodes)
        .collect();
    let model = ExplicitModel::new(positions, elements, vec![SILICONE; count], held).unwrap();
    let top = (edge_nodes * edge_nodes..2 * edge_nodes * edge_nodes)
        .map(|v| u32::try_from(v).unwrap())
        .collect();
    (model, top)
}

/// [`slab`] moved by `offset`.
fn moved_slab(n: usize, h: f64, offset: [f64; 3]) -> (ExplicitModel, Vec<u32>) {
    let (model, top) = slab(n, h, 0.0);
    let positions = model
        .rest_positions()
        .iter()
        .map(|p| [p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]])
        .collect();
    let count = model.elements().len();
    let moved = ExplicitModel::new(
        positions,
        model.elements().to_vec(),
        vec![SILICONE; count],
        model.held().to_vec(),
    )
    .unwrap();
    (moved, top)
}

/// A flat obstacle lying on the slab's top and pressing down: outward normal
/// `−z` everywhere.
fn plate() -> Obstacle {
    let (grid, values) = bake([-0.04, -0.04, -0.004], [0.04, 0.04, 0.004], 0.004, |p| {
        -p[2]
    })
    .unwrap();
    Obstacle {
        grid,
        values,
        start: 0.0,
        interval: 1.0,
        poses: vec![IDENTITY],
        friction: 0.0,
    }
}

/// A window of 4 steps at rest, with node `n` pressing `forces[n]`.
fn window(model: &ExplicitModel, forces: &[f64]) -> Snapshot {
    let nodes = model.node_count();
    Snapshot {
        displacements: vec![[0.0; 3]; nodes],
        velocities: vec![[0.0; 3]; nodes],
        anchors: vec![[0.0; 3]; nodes],
        displacement_sums: vec![[0.0; 3]; nodes],
        normal_force_sums: forces.iter().map(|f| 4.0 * f).collect(),
        friction_sums: vec![[0.0; 3]; nodes],
        accumulated_steps: 4,
    }
}

/// Each top node's share of the top face: a third of each incident top
/// triangle, found without the readings' code.
fn top_areas(model: &ExplicitModel, top: &[u32]) -> Vec<f64> {
    let on_top = |v: u32| top.contains(&v);
    let rest = model.rest_positions();
    let mut areas = vec![0.0; model.node_count()];
    for corners in model.surface_triangles() {
        if corners.iter().all(|&v| on_top(v)) {
            let [a, b, c] = corners.map(|v| rest[v as usize]);
            for &v in corners {
                areas[v as usize] += triangle_area(a, b, c) / 3.0;
            }
        }
    }
    areas
}

/// Every top node pressing with `pressure` over its share of the top.
fn uniform(model: &ExplicitModel, top: &[u32], pressure: f64) -> Vec<f64> {
    let areas = top_areas(model, top);
    let mut forces = vec![0.0; model.node_count()];
    for &v in top {
        forces[v as usize] = pressure * areas[v as usize];
    }
    forces
}

#[test]
fn the_window_reads_mean_positions_and_forces() {
    let (model, top) = slab(2, 2.0 * MM, 0.0);
    let mut snapshot = window(&model, &uniform(&model, &top, 1000.0));
    // The last instant is elsewhere; the window's mean is 0.1 mm up.
    snapshot.displacements = vec![[5.0, 5.0, 5.0]; model.node_count()];
    snapshot.displacement_sums = vec![[0.0, 0.0, 4.0 * 0.1 * MM]; model.node_count()];
    let contact = WindowContact::read(&model, &snapshot, &plate(), 0.0);
    for (v, p) in contact.positions().iter().enumerate() {
        let rest = model.rest_positions()[v];
        assert_eq!(*p, [rest[0], rest[1], rest[2] + 0.1 * MM]);
    }
    let forces = uniform(&model, &top, 1000.0);
    for (read, set) in contact.forces().iter().zip(&forces) {
        assert!((read - set).abs() <= 1e-15 * set, "{read} vs {set}");
    }
}

#[test]
fn an_edge_node_takes_only_the_face_the_obstacle_touches() {
    let (model, top) = slab(4, 2.0 * MM, 0.0);
    let snapshot = window(&model, &uniform(&model, &top, 1000.0));
    let contact = WindowContact::read(&model, &snapshot, &plate(), 0.0);
    let expected = top_areas(&model, &top);
    for &v in &top {
        let v = v as usize;
        assert!(
            (contact.areas()[v] - expected[v]).abs() < 1e-18,
            "node {v}: {} vs {}",
            contact.areas()[v],
            expected[v]
        );
    }
    // A third of every incident triangle also counts the corner's side
    // faces, which the contact area leaves out.
    let corner = top[0];
    let rest = model.rest_positions();
    let sides: f64 = model
        .surface_triangles()
        .iter()
        .filter(|t| t.contains(&corner) && !t.iter().all(|v| top.contains(v)))
        .map(|&t| {
            let [a, b, c] = t.map(|v| rest[v as usize]);
            triangle_area(a, b, c) / 3.0
        })
        .sum();
    let corner = corner as usize;
    assert!(sides > 0.0);
    let all = tributary_area(&model, &snapshot, corner);
    assert!(
        (all - (contact.areas()[corner] + sides)).abs() < 1e-18,
        "{all} vs {} + {sides}",
        contact.areas()[corner]
    );
    let total: f64 = contact.areas().iter().sum();
    assert!((total / (8.0 * MM).powi(2) - 1.0).abs() < 1e-12, "{total}");
    // Off the contact there is no area.
    assert!(
        (0..model.node_count())
            .filter(|v| !top.contains(&u32::try_from(*v).unwrap()))
            .all(|v| contact.areas()[v] == 0.0)
    );
}

#[test]
fn a_uniform_pressure_reads_its_pressure_on_an_irregular_mesh() {
    let (model, top) = slab(30, 2.0 * MM, 0.2);
    let snapshot = window(&model, &uniform(&model, &top, 1000.0));
    let contact = WindowContact::read(&model, &snapshot, &plate(), 0.0);
    assert!(
        contact
            .pressures()
            .iter()
            .all(|&(p, _)| (p / 1000.0 - 1.0).abs() < 1e-12)
    );
    let patch = contact.patch_peak(PROBE_AREA).unwrap();
    assert!(
        (patch.pressure / 1000.0 - 1.0).abs() < 2e-3,
        "{}",
        patch.pressure
    );
    // Its centre is at least a patch radius from the slab's edge.
    let radius = (PROBE_AREA / PI).sqrt();
    assert!(patch.centre[0].abs().max(patch.centre[1].abs()) < 0.03 - radius);
    // A patch at the corner holds a quarter of one.
    let corner = contact.patch_at(PROBE_AREA, [-0.03, -0.03, 0.0]);
    assert!((corner / 250.0 - 1.0).abs() < 1e-3, "{corner}");
}

#[test]
fn a_point_force_reads_its_force_over_the_probe_area() {
    let (model, top) = slab(30, 2.0 * MM, 0.2);
    let middle = top[top.len() / 2] as usize;
    let mut forces = vec![0.0; model.node_count()];
    forces[middle] = 0.3;
    let snapshot = window(&model, &forces);
    let contact = WindowContact::read(&model, &snapshot, &plate(), 0.0);
    let patch = contact.patch_peak(PROBE_AREA).unwrap();
    assert!(
        (patch.pressure / (0.3 / PROBE_AREA) - 1.0).abs() < 1e-12,
        "{}",
        patch.pressure
    );
    // Two patch radii away, nothing.
    let radius = (PROBE_AREA / PI).sqrt();
    let p = contact.positions()[middle];
    let away = contact.patch_at(PROBE_AREA, [p[0] + 2.0 * radius, p[1], 0.0]);
    assert_eq!(away, 0.0);
}

#[test]
fn a_patch_reads_the_same_wherever_the_mesh_sits_in_the_world() {
    // A point force whose spread straddles the patch's rim, read with the
    // whole slab moved across the world in steps of a fortieth of a radius:
    // the points the patch finds must not depend on where the origin is.
    let radius = (PROBE_AREA / PI).sqrt();
    let read = |shift: f64| {
        let (model, top) = moved_slab(8, 2.0 * MM, [shift, 0.0, 0.0]);
        let loaded = top[top.len() / 2] as usize;
        let mut forces = vec![0.0; model.node_count()];
        forces[loaded] = 0.3;
        let contact = WindowContact::read(&model, &window(&model, &forces), &plate(), 0.0);
        let p = contact.positions()[loaded];
        contact.patch_at(PROBE_AREA, [p[0] + radius, p[1], p[2]])
    };
    let first = read(0.0);
    assert!(first > 0.0 && first < 0.3 / PROBE_AREA, "{first}");
    for step in 1..=40 {
        let shifted = read(f64::from(step) * radius / 40.0);
        assert!(
            (shifted / first - 1.0).abs() < 1e-9,
            "shift {step}: {shifted} vs {first}"
        );
    }
}

#[test]
fn on_a_coarse_mesh_the_peak_can_centre_on_a_triangle() {
    // Cells wider than the patch, and one top triangle's three nodes pressed:
    // a patch on the triangle's centroid holds more than one on any node.
    let (model, top) = slab(6, 6.0 * MM, 0.0);
    let pressed = model
        .surface_triangles()
        .iter()
        .find(|t| t.iter().all(|v| top.contains(v)))
        .copied()
        .unwrap();
    let mut forces = vec![0.0; model.node_count()];
    for v in pressed {
        forces[v as usize] = 0.1;
    }
    let contact = WindowContact::read(&model, &window(&model, &forces), &plate(), 0.0);
    let peak = contact.patch_peak(PROBE_AREA).unwrap();
    let on_nodes = pressed
        .iter()
        .map(|&v| contact.patch_at(PROBE_AREA, contact.positions()[v as usize]))
        .fold(0.0, f64::max);
    assert!(
        peak.pressure > 1.01 * on_nodes,
        "{} vs {on_nodes}",
        peak.pressure
    );
    let [a, b, c] = pressed.map(|v| contact.positions()[v as usize]);
    for axis in 0..3 {
        let centroid = (a[axis] + b[axis] + c[axis]) / 3.0;
        assert!((peak.centre[axis] - centroid).abs() < 1e-15);
    }

    // Two of its nodes pressed, the third not: the patch on the triangle's
    // centroid still holds more than one on any node.
    let mut forces = vec![0.0; model.node_count()];
    for &v in &pressed[..2] {
        forces[v as usize] = 0.1;
    }
    let contact = WindowContact::read(&model, &window(&model, &forces), &plate(), 0.0);
    let peak = contact.patch_peak(PROBE_AREA).unwrap();
    let on_nodes = pressed[..2]
        .iter()
        .map(|&v| contact.patch_at(PROBE_AREA, contact.positions()[v as usize]))
        .fold(0.0, f64::max);
    assert!(
        peak.pressure > 1.01 * on_nodes,
        "{} vs {on_nodes}",
        peak.pressure
    );
}

#[test]
fn half_a_patch_off_the_contact_reads_half() {
    // Pressed where x ≤ 0 only: the force falls off across the one cell
    // beyond, so the contact's edge is halfway across it.
    let h = 1.0 * MM;
    let (model, top) = slab(60, h, 0.0);
    let mut forces = uniform(&model, &top, 1000.0);
    for &v in &top {
        if model.rest_positions()[v as usize][0] > 1e-9 {
            forces[v as usize] = 0.0;
        }
    }
    let snapshot = window(&model, &forces);
    let contact = WindowContact::read(&model, &snapshot, &plate(), 0.0);
    // The unpressed nodes face the plate too, and have no contact area.
    assert!(
        top.iter()
            .filter(|&&v| forces[v as usize] == 0.0)
            .all(|&v| contact.areas()[v as usize] == 0.0)
    );
    let edge = contact.patch_at(PROBE_AREA, [0.5 * h, 0.0, 0.0]);
    assert!((edge / 500.0 - 1.0).abs() < 2e-3, "{edge}");
    let peak = contact.patch_peak(PROBE_AREA).unwrap();
    assert!(
        (peak.pressure / 1000.0 - 1.0).abs() < 2e-3,
        "{}",
        peak.pressure
    );
}

#[test]
fn a_stretched_window_reads_its_stretched_area() {
    // The window's mean state is the rest state stretched 1.3× along x: every
    // top triangle is 1.3× its rest area, and a node's contact area with it.
    let (model, top) = slab(30, 2.0 * MM, 0.2);
    let rest = top_areas(&model, &top);
    let stretched = |forces: Vec<f64>| {
        let mut snapshot = window(&model, &forces);
        snapshot.displacement_sums = model
            .rest_positions()
            .iter()
            .map(|p| [4.0 * 0.3 * p[0], 0.0, 0.0])
            .collect();
        WindowContact::read(&model, &snapshot, &plate(), 0.0)
    };
    let mut forces = vec![0.0; model.node_count()];
    for &v in &top {
        forces[v as usize] = 1000.0 * 1.3 * rest[v as usize];
    }
    let contact = stretched(forces);
    for &v in &top {
        let (read, expected) = (contact.areas()[v as usize], 1.3 * rest[v as usize]);
        assert!(
            (read / expected - 1.0).abs() < 1e-12,
            "{read} vs {expected}"
        );
    }
    assert!(
        contact
            .pressures()
            .iter()
            .all(|&(p, _)| (p / 1000.0 - 1.0).abs() < 1e-12)
    );
    let patch = contact.patch_peak(PROBE_AREA).unwrap();
    assert!(
        (patch.pressure / 1000.0 - 1.0).abs() < 2e-3,
        "{}",
        patch.pressure
    );
    // A point force in the stretched window is all found by a patch on it.
    let middle = top[top.len() / 2] as usize;
    let mut forces = vec![0.0; model.node_count()];
    forces[middle] = 0.3;
    let patch = stretched(forces).patch_peak(PROBE_AREA).unwrap();
    assert!(
        (patch.pressure / (0.3 / PROBE_AREA) - 1.0).abs() < 1e-12,
        "{}",
        patch.pressure
    );
}

#[test]
fn the_contact_area_takes_the_obstacles_normal_as_posed_at_the_window() {
    // The plate is level at t = 0 and turned 30° about y at t = 1, when the
    // window is read: an interior top node's area is cos 30° of its share.
    let (model, top) = slab(4, 2.0 * MM, 0.0);
    let turn = 30.0_f64.to_radians();
    let turned = Pose {
        qw: (turn / 2.0).cos(),
        qy: (turn / 2.0).sin(),
        ..IDENTITY
    };
    let obstacle = Obstacle {
        poses: vec![IDENTITY, turned],
        ..plate()
    };
    let snapshot = window(&model, &uniform(&model, &top, 1000.0));
    let contact = WindowContact::read(&model, &snapshot, &obstacle, 1.0);
    let shares = top_areas(&model, &top);
    let interior = top.iter().filter(|&&v| {
        let p = model.rest_positions()[v as usize];
        p[0].abs() < 3.0 * MM && p[1].abs() < 3.0 * MM
    });
    for &v in interior {
        let (read, expected) = (contact.areas()[v as usize], turn.cos() * shares[v as usize]);
        assert!(
            (read / expected - 1.0).abs() < 1e-12,
            "{read} vs {expected}"
        );
    }
}

/// A flat obstacle whose outward normal is `normal` everywhere, over the
/// plate's box.
fn plane(normal: [f64; 3]) -> Obstacle {
    let (grid, values) = bake([-0.04, -0.04, -0.004], [0.04, 0.04, 0.004], 0.004, |p| {
        normal[0] * p[0] + normal[1] * p[1] + normal[2] * p[2]
    })
    .unwrap();
    Obstacle {
        grid,
        values,
        ..plate()
    }
}

/// A third of each surface triangle whose rest corners all satisfy `on`,
/// summed at each node.
fn face_areas(model: &ExplicitModel, on: impl Fn([f64; 3]) -> bool) -> Vec<f64> {
    let rest = model.rest_positions();
    let mut areas = vec![0.0; model.node_count()];
    for corners in model.surface_triangles() {
        if corners.iter().all(|&v| on(rest[v as usize])) {
            let [a, b, c] = corners.map(|v| rest[v as usize]);
            for &v in corners {
                areas[v as usize] += triangle_area(a, b, c) / 3.0;
            }
        }
    }
    areas
}

#[test]
fn a_face_turned_away_from_the_obstacle_adds_nothing() {
    // A plane pressing down and toward −x: its normal is (−0.6, 0, −0.8). The
    // top faces it at 0.8 and the +x side at 0.6; the −x side is turned away.
    let (model, top) = slab(4, 2.0 * MM, 0.0);
    let snapshot = window(&model, &uniform(&model, &top, 1000.0));
    let contact = WindowContact::read(&model, &snapshot, &plane([-0.6, 0.0, -0.8]), 0.0);
    let half = 4.0 * MM;
    let tops = top_areas(&model, &top);
    let plus_x = face_areas(&model, |p| (p[0] - half).abs() < 1e-12);
    let rest = model.rest_positions();
    let corner = |x: f64| {
        top.iter()
            .map(|&v| v as usize)
            .find(|&v| (rest[v][0] - x).abs() < 1e-12 && (rest[v][1] + half).abs() < 1e-12)
            .unwrap()
    };
    let (minus, plus) = (corner(-half), corner(half));
    assert!((contact.areas()[minus] - 0.8 * tops[minus]).abs() < 1e-18);
    let expected = 0.8 * tops[plus] + 0.6 * plus_x[plus];
    assert!((contact.areas()[plus] - expected).abs() < 1e-18);
}

#[test]
fn a_node_the_obstacle_meets_side_on_keeps_its_force() {
    // A wall's normal lies in the top's plane, so no face turns toward it:
    // the pressed node takes its whole share of the top, and keeps its force.
    let (model, top) = slab(4, 2.0 * MM, 0.0);
    let middle = top[top.len() / 2] as usize;
    let mut forces = vec![0.0; model.node_count()];
    forces[middle] = 0.3;
    let contact = WindowContact::read(
        &model,
        &window(&model, &forces),
        &plane([-1.0, 0.0, 0.0]),
        0.0,
    );
    let share = top_areas(&model, &top)[middle];
    assert_eq!(contact.pressures(), vec![(0.3 / share, share)]);
    let patch = contact.patch_peak(PROBE_AREA).unwrap();
    assert!(
        (patch.pressure / (0.3 / PROBE_AREA) - 1.0).abs() < 1e-12,
        "{}",
        patch.pressure
    );
}

/// The area of a cylinder of radius `radius` inside a ball of radius `ball`
/// centred on it, over the ball's disc `π ball²`: the patch's excess on a
/// bore. Along the circumference, arc `s` is chord `2 r sin(s / 2r)` away.
fn ball_on_cylinder(radius: f64, ball: f64) -> f64 {
    let reach = 2.0 * radius * (ball / (2.0 * radius)).asin();
    let steps = 200_000;
    let ds = 2.0 * reach / f64::from(steps);
    let area: f64 = (0..steps)
        .map(|i| {
            let s = -reach + (f64::from(i) + 0.5) * ds;
            let chord = 2.0 * radius * (s / (2.0 * radius)).sin();
            2.0 * (ball * ball - chord * chord).max(0.0).sqrt() * ds
        })
        .sum();
    area / (PI * ball * ball)
}

#[test]
fn on_a_bore_the_ball_holds_a_little_more_than_the_probe_area() {
    let tube = Tube::plan(Mesh::Cells {
        radial: 1,
        circumferential: 256,
        axial: 120,
    });
    let model = tube.model(SILICONE, Walls::Free).unwrap();
    // A mandrel exactly in the bore, baked around the middle of the tube.
    let bore = tube.inner_radius;
    let (grid, values) = bake([-0.012, -0.012, 0.030], [0.012, 0.012, 0.090], 0.001, |p| {
        p[0].hypot(p[1]) - bore
    })
    .unwrap();
    // The window finds the tube 2 mm along x, and the mandrel posed there.
    let shift = 0.002;
    let mandrel = Obstacle {
        grid,
        values,
        start: 0.0,
        interval: 1.0,
        poses: vec![Pose {
            tx: shift,
            ..IDENTITY
        }],
        friction: 0.0,
    };
    // Every bore node from 40 to 80 mm pressing 1 kPa over its share of the
    // bore.
    let pressure = 1000.0;
    let mut forces = vec![0.0; model.node_count()];
    let rest = model.rest_positions();
    let chord = 2.0 * bore * (PI / tube.circumferential as f64).sin();
    let spacing = tube.length / tube.axial as f64;
    for (v, p) in rest.iter().enumerate() {
        let (i, _, _) = tube.levels(v);
        if i == 0 && (0.040..=0.080).contains(&p[2]) {
            forces[v] = pressure * chord * spacing;
        }
    }
    let mut snapshot = window(&model, &forces);
    snapshot.displacement_sums = vec![[4.0 * shift, 0.0, 0.0]; model.node_count()];
    let contact = WindowContact::read(&model, &snapshot, &mandrel, 0.0);
    // Each facet is half a cell's turn off its nodes' normals.
    let facet = (PI / tube.circumferential as f64).cos();
    assert!(
        contact
            .pressures()
            .iter()
            .all(|&(p, _)| (p * facet / pressure - 1.0).abs() < 1e-4)
    );
    let centre = [bore + shift, 0.0, 0.060];
    let read = contact.patch_at(PROBE_AREA, centre) / pressure;
    let expected = ball_on_cylinder(bore, (PROBE_AREA / PI).sqrt());
    assert!(expected > 1.005 && expected < 1.02, "{expected}");
    assert!((read / expected - 1.0).abs() < 1e-3, "{read} vs {expected}");
}

#[test]
fn the_normal_follows_the_obstacles_pose() {
    // Turned about y by 0 at t = 0 and by a quarter turn at t = 1: halfway,
    // an eighth of a turn.
    let quarter = Pose {
        qw: FRAC_1_SQRT_2,
        qy: FRAC_1_SQRT_2,
        tz: 0.002,
        ..IDENTITY
    };
    let obstacle = Obstacle {
        poses: vec![IDENTITY, quarter],
        ..plate()
    };
    let at = |t: f64| obstacle.world_normal(t, [0.0, 0.0, 0.001]);
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12);
    assert!(close(at(0.0), [0.0, 0.0, -1.0]), "{:?}", at(0.0));
    assert!(close(at(1.0), [-1.0, 0.0, 0.0]), "{:?}", at(1.0));
    assert!(
        close(at(0.5), [-FRAC_1_SQRT_2, 0.0, -FRAC_1_SQRT_2]),
        "{:?}",
        at(0.5)
    );
    assert!((obstacle.pose_at(0.25).tz - 0.0005).abs() < 1e-15);

    // A bore's axis moved 3 mm along x: the normal is read about the moved
    // axis, not the world's.
    let (grid, values) = bake([-0.016, -0.016, 0.0], [0.016, 0.016, 0.004], 0.001, |p| {
        p[0].hypot(p[1]) - 0.010
    })
    .unwrap();
    let moved = Obstacle {
        grid,
        values,
        start: 0.0,
        interval: 1.0,
        poses: vec![Pose {
            tx: 0.003,
            ..IDENTITY
        }],
        friction: 0.0,
    };
    let normal = moved.world_normal(0.0, [0.003, 0.010, 0.002]);
    assert!(close(normal, [0.0, 1.0, 0.0]), "{normal:?}");
}
