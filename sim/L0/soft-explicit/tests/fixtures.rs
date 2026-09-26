//! The tube fixture: its mesh, the mandrel, the insertion, the band readout,
//! and the oracle's golden values.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

use std::f64::consts::{PI, TAU};

use sim_soft_explicit::ModelError;
use sim_soft_explicit::executor::Snapshot;
use sim_soft_explicit::f64::Material;
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::grid::BakeError;
use sim_soft_explicit::fixtures::tube::{
    BandReading, ECOFLEX_00_30_VISCOUS_TIME, Insertion, Mandrel, Mesh, Tube, TubeRun, Walls,
    node_pressures, read_band,
};

const SILICONE: Material = Material {
    mu: 23.0e3,
    lambda: 23.0e3 * 2.0 * 0.49 / (1.0 - 2.0 * 0.49),
    c2: 0.0,
    viscosity: 0.0,
    density: 1070.0,
};

#[test]
fn the_planned_meshes_have_the_planned_sizes() {
    for (mesh, elements) in [
        (Mesh::TenK, 9_792),
        (Mesh::FiftyK, 50_400),
        (Mesh::HundredK, 99_072),
    ] {
        let tube = Tube::plan(mesh);
        assert_eq!(tube.element_count(), elements);
        assert_eq!(tube.elements().len(), elements);
    }
}

#[test]
fn the_mesh_fills_the_polygonal_annulus_with_nodes_on_circles() {
    let tube = Tube::plan(Mesh::TenK);
    let model = tube.model(SILICONE, Walls::Free).unwrap();
    // The exact volume of the n-gon annulus the mesh discretizes.
    let n = tube.circumferential as f64;
    let polygon = 0.5 * n * (TAU / n).sin();
    let volume = polygon * (tube.outer_radius.powi(2) - tube.inner_radius.powi(2)) * tube.length;
    let meshed: f64 = model.rest_volumes().iter().sum();
    assert!(
        (meshed / volume - 1.0).abs() < 1e-12,
        "{meshed} vs {volume}"
    );
    assert!(meshed < PI * (tube.outer_radius.powi(2) - tube.inner_radius.powi(2)) * tube.length);
    for (node, p) in model.rest_positions().iter().enumerate() {
        let (i, _, _) = tube.levels(node);
        let (r, _, _) = tube.cylindrical(i, 0, 0);
        assert!(
            (p[0].hypot(p[1]) - r).abs() < 1e-15,
            "node {node} off its circle"
        );
    }
    // The far end is held and nothing else.
    let held = model.held().iter().filter(|&&h| h).count();
    assert_eq!(held, (tube.radial + 1) * tube.circumferential);
    // Every band node is on the surface.
    let band = tube.band(0.020, 0.060);
    assert!(
        band.iter()
            .all(|&n| !model.surface_incidence().of(n as usize).is_empty())
    );
    let levels: std::collections::BTreeSet<usize> =
        band.iter().map(|&n| tube.levels(n as usize).2).collect();
    assert_eq!(band.len(), levels.len() * tube.circumferential);
}

#[test]
fn the_cased_tube_is_held_whole_outside_and_axially_everywhere() {
    let tube = Tube::plan(Mesh::TenK);
    let model = tube.model(SILICONE, Walls::Cased).unwrap();
    for (node, &[first, second]) in model.constraints().iter().enumerate() {
        let (i, _, _) = tube.levels(node);
        assert_eq!(model.held()[node], i == tube.radial);
        assert_eq!(first, [0.0, 0.0, 1.0]);
        assert_eq!(second, [0.0; 3]);
    }
}

#[test]
fn the_mandrel_is_a_cylinder_with_a_round_nose() {
    let m = Mandrel { radius: 0.011 };
    assert_eq!(m.distance([0.0, 0.0, 0.0]), 0.0);
    assert!((m.distance([0.0, 0.0, 0.004]) - 0.004).abs() < 1e-15);
    assert!((m.distance([0.015, 0.0, -0.05]) - 0.004).abs() < 1e-15);
    assert!((m.distance([0.0, 0.0, -0.05]) + 0.011).abs() < 1e-15);
    // On the nose, 45° off the axis.
    let c = [0.0, 0.0, -0.011];
    let s = std::f64::consts::FRAC_1_SQRT_2 * 0.011;
    assert!(m.distance([s, 0.0, c[2] + s]).abs() < 1e-15);
    let (grid, values) = m
        .baked([-0.02, -0.02, -0.03], [0.02, 0.02, 0.005], 0.0005)
        .unwrap();
    assert_eq!(
        values.len(),
        (grid.size_x * grid.size_y * grid.size_z) as usize
    );
    let at = |i: u32, j: u32, k: u32| values[((k * grid.size_y + j) * grid.size_x + i) as usize];
    let p = |i: u32, j: u32, k: u32| {
        [
            grid.origin_x + f64::from(i) * grid.cell_size,
            grid.origin_y + f64::from(j) * grid.cell_size,
            grid.origin_z + f64::from(k) * grid.cell_size,
        ]
    };
    for (i, j, k) in [(0, 0, 0), (40, 40, 60), (13, 57, 69), (80, 3, 2)] {
        assert_eq!(at(i, j, k), m.distance(p(i, j, k)));
    }
}

#[test]
fn the_insertion_ramps_holds_speed_and_stops_at_depth() {
    let insertion = Insertion::plan(1.0);
    assert!((insertion.tip(0.0) + 0.005).abs() < 1e-15);
    assert!((insertion.tip(1.0) - 0.100).abs() < 1e-15);
    assert_eq!(insertion.tip(1.1), insertion.tip(1.0));
    // The speed at mid-run is the top speed, and the profile is continuous
    // at both ends of the constant phase.
    let top = 0.105 / 0.9;
    let speed = |t: f64| (insertion.tip(t + 1e-7) - insertion.tip(t - 1e-7)) / 2e-7;
    assert!((speed(0.5) / top - 1.0).abs() < 1e-6);
    for t in [0.1, 0.9] {
        assert!((insertion.tip(t + 1e-9) - insertion.tip(t - 1e-9)).abs() < 1e-9);
        assert!((speed(t) / top - 1.0).abs() < 1e-3);
    }
    let obstacle = insertion
        .obstacle(
            Mandrel { radius: 0.011 },
            &Tube::plan(Mesh::TenK),
            0.0005,
            0.0,
        )
        .unwrap();
    assert_eq!(obstacle.poses.len(), 1001);
    assert!((obstacle.poses[1000].tz - 0.100).abs() < 1e-15);
}

/// The tributary area of a band node on a wall at radius `radius` stretched
/// axially by `stretch`, in closed form: one chord `2 r sin(π / n_θ)` times
/// one stretched level spacing. Each node is a corner of six triangles of
/// half a quad each, and takes a third of each.
fn band_area(tube: &Tube, radius: f64, stretch: f64) -> f64 {
    let chord = 2.0 * radius * (PI / tube.circumferential as f64).sin();
    chord * stretch * tube.length / tube.axial as f64
}

/// A window's snapshot, held for `steps` steps: the inner wall moved out to
/// `radius`, everything stretched axially by `stretch`, and every band node
/// pressing with `pressure` over its closed-form tributary area.
fn uniform(
    tube: &Tube,
    pressure: f64,
    radius: f64,
    stretch: f64,
    steps: u64,
) -> (Snapshot, BandReading) {
    let model = tube.model(SILICONE, Walls::Free).unwrap();
    let displacements: Vec<[f64; 3]> = model
        .rest_positions()
        .iter()
        .map(|p| {
            let r = p[0].hypot(p[1]);
            let scale = if (r - tube.inner_radius).abs() < 1e-12 {
                radius / r - 1.0
            } else {
                0.0
            };
            [scale * p[0], scale * p[1], (stretch - 1.0) * p[2]]
        })
        .collect();
    let n = steps as f64;
    let band = tube.band(0.020, 0.060);
    let mut normal_force_sums = vec![0.0; model.node_count()];
    for &node in &band {
        normal_force_sums[node as usize] = pressure * band_area(tube, radius, stretch) * n;
    }
    // The end state is at rest: the readout must read the window's means,
    // not the last instant.
    let snapshot = Snapshot {
        displacement_sums: displacements.iter().map(|u| u.map(|c| c * n)).collect(),
        displacements: vec![[0.0; 3]; model.node_count()],
        velocities: vec![[0.0; 3]; model.node_count()],
        anchors: vec![[0.0; 3]; model.node_count()],
        friction_sums: vec![[0.0; 3]; model.node_count()],
        normal_force_sums,
        accumulated_steps: steps,
    };
    let reading = read_band(tube, &model, &snapshot, Mandrel { radius }, &band);
    (snapshot, reading)
}

#[test]
fn a_uniform_band_reads_its_pressure_gap_and_stretch() {
    let tube = Tube::plan(Mesh::TenK);
    let (snapshot, reading) = uniform(&tube, 3000.0, 0.011, 0.98, 40);
    assert!(
        (reading.pressure / 3000.0 - 1.0).abs() < 1e-12,
        "{}",
        reading.pressure
    );
    assert!(
        reading
            .paired_levels
            .iter()
            .all(|p| (p / 3000.0 - 1.0).abs() < 1e-12)
    );
    assert!(reading.node_scatter < 1e-12);
    let model = tube.model(SILICONE, Walls::Free).unwrap();
    let band = tube.band(0.020, 0.060);
    assert!(
        node_pressures(&model, &snapshot, &band)
            .iter()
            .all(|p| (p / 3000.0 - 1.0).abs() < 1e-12)
    );
    assert!(reading.gap.abs() < 1e-15, "{}", reading.gap);
    assert!((reading.axial_stretch - 0.98).abs() < 1e-12);
}

#[test]
fn the_errors_vanish_at_the_oracle_and_the_gap_moves_only_the_corrected_one() {
    let tube = Tube::plan(Mesh::TenK);
    let case = THICK_TUBE[0];
    let a = case.mandrel_ratio * tube.inner_radius;
    let p = case.pressure_over_mu * SILICONE.mu;
    let (_, mut reading) = uniform(&tube, p, a, case.axial_stretch, 10);
    let errors = case.errors(&reading, tube.inner_radius, SILICONE.mu);
    assert!(
        errors.raw.abs() < 1e-10 && errors.gap_corrected.unwrap().abs() < 1e-10,
        "{errors:?}"
    );
    // Nodes 0.05 mm inside the mandrel: the corrected reference drops by the
    // linearized amount, the raw one does not move.
    reading.gap = -0.00005;
    let errors = case.errors(&reading, tube.inner_radius, SILICONE.mu);
    assert!(errors.raw.abs() < 1e-10);
    let expected = case.pressure_over_mu
        / (case.pressure_over_mu - case.pressure_per_mandrel_ratio * 0.005)
        - 1.0;
    assert!((errors.gap_corrected.unwrap() - expected).abs() < 1e-12);
    // The cased tube reports no gap-corrected error.
    let cased = THICK_TUBE[4].errors(&reading, tube.inner_radius, SILICONE.mu);
    assert_eq!(cased.gap_corrected, None);
}

#[test]
fn the_golden_values_are_the_plans() {
    let plan = [
        (0.12352, 0.98523),
        (0.12365, 0.98511),
        (0.30507, 0.95779),
        (0.30544, 0.95747),
    ];
    for (case, (p, lz)) in THICK_TUBE.iter().zip(plan) {
        assert_eq!(case.walls, Walls::Free);
        assert!((case.pressure_over_mu - p).abs() <= 0.5e-5, "{case:?}");
        assert!((case.axial_stretch - lz).abs() <= 0.5e-5, "{case:?}");
        assert!(case.pressure_per_mandrel_ratio > 0.0 && case.pressure_per_axial_stretch > 0.0);
    }
    let cased = THICK_TUBE[4];
    assert_eq!(cased.walls, Walls::Cased);
    assert!((cased.pressure_over_mu - 4.1417).abs() <= 0.5e-4);
    assert_eq!(cased.axial_stretch, 1.0);
    assert!(THICK_TUBE[..5].iter().all(|case| case.c2_over_mu == 0.0));
    // The Yeoh case (plan 16h): ECOFLEX_00_30's C₂/μ on the λ_a 1.3 corner,
    // 4.22 % above its neo-Hookean pressure.
    let yeoh = THICK_TUBE[5];
    assert_eq!(yeoh.walls, Walls::Free);
    assert_eq!(
        (yeoh.mandrel_ratio, yeoh.poisson),
        (THICK_TUBE[2].mandrel_ratio, THICK_TUBE[2].poisson)
    );
    assert!((yeoh.c2_over_mu - 2050.0 / 23000.0).abs() <= 1e-15);
    let rise = yeoh.pressure_over_mu / THICK_TUBE[2].pressure_over_mu - 1.0;
    assert!((rise - 0.0422).abs() <= 0.5e-4, "{rise}");
}

#[test]
fn a_runs_yeoh_term_comes_from_its_case_and_its_viscosity_from_its_time() {
    let run = |case| TubeRun {
        mesh: Mesh::TenK,
        case,
        mu: 23.0e3,
        viscous_time: ECOFLEX_00_30_VISCOUS_TIME,
        density: 1070.0,
        insertion: Insertion::plan(1.0),
        window: 0.1,
        friction: 0.0,
        grid_cell: 0.0005,
    };
    assert_eq!(run(THICK_TUBE[2]).material().c2, 0.0);
    assert!((run(THICK_TUBE[5]).material().c2 - 2050.0).abs() <= 1e-9);
    // Ecoflex 00-30's viscosity, 7 Pa·s (plan §16p); C₂ and η both scale with
    // μ, which stiffness scaling (15d.10) relies on.
    assert!((run(THICK_TUBE[2]).material().viscosity - 7.0).abs() <= 1e-12);
    let stiffer = TubeRun {
        mu: 46.0e3,
        ..run(THICK_TUBE[5])
    }
    .material();
    assert!((stiffer.c2 - 4100.0).abs() <= 1e-9 && (stiffer.viscosity - 14.0).abs() <= 1e-12);
}

#[test]
fn a_bad_bake_or_an_oversized_tube_is_refused() {
    let m = Mandrel { radius: 0.011 };
    let (low, high) = ([-0.01, -0.01, -0.01], [0.01, 0.01, 0.01]);
    assert!(matches!(m.baked(low, high, 0.0), Err(BakeError::Cell)));
    assert!(matches!(m.baked(high, low, 0.001), Err(BakeError::Box)));
    assert!(matches!(
        m.baked(low, [0.0103, 0.01, 0.01], 0.001),
        Err(BakeError::NotWholeCells { axis: 0 })
    ));
    assert!(matches!(
        m.baked([-1.0; 3], [1.0; 3], 1e-4),
        Err(BakeError::TooLarge)
    ));
    let (grid, _) = m.baked(low, high, 0.001).unwrap();
    assert_eq!((grid.size_x, grid.size_y, grid.size_z), (21, 21, 21));
    let huge = Tube {
        radial: 1 << 20,
        circumferential: 1 << 12,
        axial: 1 << 4,
        ..Tube::plan(Mesh::TenK)
    };
    assert!(matches!(
        huge.model(SILICONE, Walls::Free),
        Err(ModelError::TooLarge)
    ));
}
