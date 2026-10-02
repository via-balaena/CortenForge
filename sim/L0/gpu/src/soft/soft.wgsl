// The soft executor's entry points (recon §17b). Appended to the shared math
// (`sim_soft_explicit::SHARED_WGSL`), which does the physics: each entry point
// is the CPU executor's loop body for one phase (`sim-soft-explicit`,
// `src/cpu/executor.rs`), over one element, node or surface node.
//
// Every binding sits in group 0 under its own number, and each entry point's
// layout lists the ones it uses (`src/soft/kernels.rs`). A binding named for
// a role (the displacements a phase reads, the terms a reduction sums) is
// bound to the step's arrays or to scratch ones, so the internal energy read
// and the step estimate run the phases without touching the step's outputs.
//
// No atomic adds a float: each gather sums a node's incidence list in its
// order, and each sum over nodes is a fixed tree, so a run repeats bit for
// bit. Counts are u32 atomics into a 64-bit pair.

// ---- Layout ----

// Invocations a workgroup, for per-item work.
const WORKGROUP: u32 = 64u;
// Invocations a workgroup, and items a partial, for a reduction.
const TREE: u32 = 256u;
// The most components a reduction's item carries.
const MOST_COMPONENTS: u32 = 7u;
// Not a surface node.
const NONE: u32 = 4294967295u;

// The counters' words: each count is a low word and a high one.
const INVERTED: u32 = 0u;
const COARSE: u32 = 2u;

// A reduction's row: the step's contact row, its boundary row, or the index.
const CONTACT_ROW: u32 = 4294967295u;
const BOUNDARY_ROW: u32 = 4294967294u;
// A reduction's operation.
const SUM: u32 = 0u;

// The power iteration's scalars.
const STOPPED: u32 = 0u;
const LARGEST: u32 = 1u;
const SIZE: u32 = 2u;
const SCALE: u32 = 3u;
const TAKEN: u32 = 4u;

// The model and the obstacle, fixed at upload.
struct Constants {
    nodes: u32,
    elements: u32,
    surface: u32,
    viscous: u32,
    @align(16) grid: SdfGridLayout,
    @align(16) fine: SdfGridLayout,
    @align(16) has_fine: u32,
    friction: f32,
}

// What a step carries (`submit::Recorder`'s ring).
struct StepValues {
    dt: f32,
    damping: f32,
    contact_row: u32,
    boundary_row: u32,
    // The obstacle's pose at the step's start and end, formed on the host.
    @align(16) start: Pose,
    @align(16) end: Pose,
    // The power iteration's finite-difference step and viscous weight.
    @align(16) perturbation: f32,
    weight: f32,
}

// A node's constants.
struct Node {
    rest: array<f32, 3>,
    // The rest position less the rest positions' centroid, the point the
    // contact moment is taken about.
    arm: array<f32, 3>,
    mass: f32,
    inverse_mass: f32,
    rest_volume: f32,
    // `λ_a − κ_a`, the λ its averaged term uses.
    lambda: f32,
    constraints: array<array<f32, 3>, 2>,
    // Its index among the surface nodes, or `NONE`.
    surface: u32,
}

// An element's constants.
struct Element {
    nodes: array<u32, 4>,
    edge_inverse: array<f32, 9>,
    rest_volume: f32,
    stabilization: f32,
    material: Material,
}

// A surface node's contact this step.
struct Contact {
    force: array<f32, 3>,
    normal_force: f32,
    friction: array<f32, 3>,
}

// A reduction: its items, each item's components, the operation, and where
// the row goes.
struct Reduction {
    items: u32,
    components: u32,
    operation: u32,
    row: u32,
}

@group(0) @binding(0) var<uniform> constants: Constants;
@group(0) @binding(1) var<uniform> step_values: StepValues;
@group(0) @binding(2) var<storage, read> node_table: array<Node>;
@group(0) @binding(3) var<storage, read> element_table: array<Element>;
@group(0) @binding(4) var<storage, read> offsets: array<u32>;
@group(0) @binding(5) var<storage, read> entries: array<u32>;
@group(0) @binding(6) var<storage, read> surface_nodes: array<u32>;
@group(0) @binding(7) var<storage, read> grid_values: array<f32>;
@group(0) @binding(8) var<storage, read> fine_map: array<u32>;
@group(0) @binding(9) var<storage, read> fine_values: array<f32>;
@group(0) @binding(10) var<storage, read_write> displacements: array<array<f32, 3>>;
@group(0) @binding(11) var<storage, read_write> velocities: array<array<f32, 3>>;
@group(0) @binding(12) var<storage, read_write> previous_velocities: array<array<f32, 3>>;
@group(0) @binding(13) var<storage, read_write> dilations: array<f32>;
@group(0) @binding(14) var<storage, read_write> volume_changes: array<f32>;
@group(0) @binding(15) var<storage, read_write> pressures: array<f32>;
@group(0) @binding(16) var<storage, read_write> element_forces: array<array<f32, 12>>;
@group(0) @binding(17) var<storage, read_write> node_forces: array<array<f32, 3>>;
@group(0) @binding(18) var<storage, read_write> elastic_forces: array<array<f32, 3>>;
@group(0) @binding(19) var<storage, read_write> viscous_forces: array<array<f32, 3>>;
@group(0) @binding(20) var<storage, read_write> contacts: array<Contact>;
@group(0) @binding(21) var<storage, read_write> anchors: array<array<f32, 3>>;
@group(0) @binding(22) var<storage, read_write> counters: array<atomic<u32>, 4>;
@group(0) @binding(23) var<storage, read_write> terms: array<f32>;
@group(0) @binding(24) var<storage, read_write> partials: array<f32>;
@group(0) @binding(25) var<storage, read_write> log_rows: array<f32>;
@group(0) @binding(26) var<uniform> reduction: Reduction;
@group(0) @binding(27) var<storage, read_write> window: array<f32>;
@group(0) @binding(28) var<storage, read_write> maxima: array<array<f32, 2>>;
@group(0) @binding(29) var<storage, read_write> scalars: array<f32>;
@group(0) @binding(30) var<storage, read> start_vector: array<array<f32, 3>>;
@group(0) @binding(31) var<storage, read_write> vector: array<array<f32, 3>>;
@group(0) @binding(32) var<storage, read_write> measured: array<array<f32, 3>>;
@group(0) @binding(33) var<storage, read_write> next_vector: array<array<f32, 3>>;
@group(0) @binding(34) var<storage, read> base_forces: array<array<f32, 3>>;
@group(0) @binding(35) var<storage, read_write> shifted: array<array<f32, 3>>;
@group(0) @binding(36) var<storage, read> shifted_forces: array<array<f32, 3>>;
@group(0) @binding(37) var<storage, read> viscous_at: array<array<f32, 3>>;
@group(0) @binding(38) var<storage, read_write> magnitudes: array<f32>;
@group(0) @binding(39) var<storage, read_write> quotients: array<f32>;
@group(0) @binding(40) var<storage, read_write> dampings: array<f32>;
// Bound read_write to every entry point and never written (no model has no
// nodes), so every pass shares a written buffer with the one before it: on the
// M4 Pro, passes that share none ran at once (recon §17e).
@group(0) @binding(41) var<storage, read_write> order: array<u32>;

// ---- Helpers ----

// Use `order`, so every entry point binds it.
fn keep_order() {
    if (constants.nodes == 0u) {
        order[0] = 0u;
    }
}

// The item an invocation takes, for a dispatch whose workgroups may run in
// two dimensions (a dimension holds at most 65 535).
fn item(id: vec3<u32>, groups: vec3<u32>, size: u32) -> u32 {
    return id.x + id.y * groups.x * size;
}

// An element's four nodes' displacements, as the shared math takes them.
fn element_displacements(corners: array<u32, 4>) -> array<f32, 12> {
    let a = displacements[corners[0]];
    let b = displacements[corners[1]];
    let c = displacements[corners[2]];
    let d = displacements[corners[3]];
    return array(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2]);
}

// An element's four nodes' velocities.
fn element_velocities(corners: array<u32, 4>) -> array<f32, 12> {
    let a = velocities[corners[0]];
    let b = velocities[corners[1]];
    let c = velocities[corners[2]];
    let d = velocities[corners[3]];
    return array(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2]);
}

// `v` without a node's held and constrained directions.
fn free_part(node: Node, v: array<f32, 3>) -> array<f32, 3> {
    if (node.inverse_mass == 0.0) {
        return array(0.0, 0.0, 0.0);
    }
    return constrain(v, node.constraints[0], node.constraints[1]);
}

// A node's contact force this step: zero off the surface.
fn contact_force(node: Node) -> array<f32, 3> {
    if (node.surface == NONE) {
        return array(0.0, 0.0, 0.0);
    }
    return contacts[node.surface].force;
}

// The largest magnitude of a vector's components.
fn largest_component(v: array<f32, 3>) -> f32 {
    return max(max(abs(v[0]), abs(v[1])), abs(v[2]));
}

var<workgroup> tally: atomic<u32>;

// Add the workgroup's `counted` invocations to count `which`, carrying into
// its high word. Called from uniform control flow by every invocation.
fn add_count(which: u32, counted: bool, local: u32) {
    if (counted) {
        atomicAdd(&tally, 1u);
    }
    workgroupBarrier();
    if (local == 0u) {
        let added = atomicLoad(&tally);
        if (added > 0u) {
            let before = atomicAdd(&counters[which], added);
            if (before + added < before) {
                atomicAdd(&counters[which + 1u], 1u);
            }
        }
    }
}

// A lookup in the obstacle's grids, and whether the fine grid answered it.
struct Lookup {
    sample: SdfSample,
    fine: bool,
}

// The lookup in the coarse grid.
fn coarse_sample(point: array<f32, 3>) -> SdfSample {
    let lattice = constants.grid;
    let coordinate = sdf_grid_coordinate(point, lattice);
    var columns = sdf_tricubic_axis(coordinate[0], lattice.size_x);
    var rows_at = sdf_tricubic_axis(coordinate[1], lattice.size_y);
    var layers = sdf_tricubic_axis(coordinate[2], lattice.size_z);
    var values: array<f32, 64>;
    for (var k = 0u; k < 4u; k++) {
        for (var j = 0u; j < 4u; j++) {
            for (var i = 0u; i < 4u; i++) {
                values[(k * 4u + j) * 4u + i] =
                    grid_values[sdf_grid_index(columns[i], rows_at[j], layers[k], lattice)];
            }
        }
    }
    return sdf_tricubic(coordinate, values, lattice);
}

// The lookup in the fine grid, where it has all 64 samples.
fn fine_sample(point: array<f32, 3>) -> Lookup {
    let missing = Lookup(SdfSample(0.0, array(0.0, 0.0, 0.0)), false);
    if (constants.has_fine == 0u) {
        return missing;
    }
    let lattice = constants.fine;
    if (!sdf_on_grid(point, lattice)) {
        return missing;
    }
    let coordinate = sdf_grid_coordinate(point, lattice);
    var columns = sdf_tricubic_axis(coordinate[0], lattice.size_x);
    var rows_at = sdf_tricubic_axis(coordinate[1], lattice.size_y);
    var layers = sdf_tricubic_axis(coordinate[2], lattice.size_z);
    var slots = sdf_stencil_bricks(columns, rows_at, layers, lattice);
    for (var s = 0u; s < 8u; s++) {
        slots[s] = fine_map[slots[s]];
    }
    if (!sdf_fine_present(slots)) {
        return missing;
    }
    var values: array<f32, 64>;
    for (var index = 0u; index < 64u; index++) {
        values[index] = fine_values[sdf_fine_index(
            columns[index % 4u], rows_at[index / 4u % 4u], layers[index / 16u],
            columns[0], rows_at[0], layers[0], slots,
        )];
    }
    return Lookup(sdf_tricubic(coordinate, values, lattice), true);
}

// The obstacle's distance and normal at a body-frame point, in the fine grid
// where it has the lookup's samples.
fn sample_obstacle(point: array<f32, 3>) -> Lookup {
    let fine = fine_sample(point);
    if (fine.fine) {
        return fine;
    }
    return Lookup(coarse_sample(point), false);
}

// ---- Phases 1–5 ----

// Phase 1: each element's dilation, counting the inverted ones.
@compute @workgroup_size(64)
fn element_dilations(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) local: u32,
) {
    keep_order();
    let e = item(id, groups, WORKGROUP);
    var inverted = false;
    if (e < constants.elements) {
        let element = element_table[e];
        let dilation = tet4_dilation(element_displacements(element.nodes), element.edge_inverse);
        dilations[e] = dilation;
        inverted = is_inverted(dilation);
    }
    add_count(INVERTED, inverted, local);
}

// Phase 2: each node's volume change, `Σ V_e (J_e − 1) / 4` over its
// incidence list in order.
@compute @workgroup_size(64)
fn gather_volume_changes(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    var sum = 0.0;
    for (var slot = offsets[a]; slot < offsets[a + 1u]; slot++) {
        let e = entries[slot] / 4u;
        sum = sum + 0.25 * element_table[e].rest_volume * dilations[e];
    }
    volume_changes[a] = sum;
}

// Phase 3: each node's averaged pressure.
@compute @workgroup_size(64)
fn nodal_pressures(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    let node = node_table[a];
    pressures[a] = pressure_lambda_term(nodal_dilation(volume_changes[a], node.rest_volume), node.lambda);
}

// Phase 4: each element's elastic forces.
@compute @workgroup_size(64)
fn elastic_element_forces(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let e = item(id, groups, WORKGROUP);
    if (e >= constants.elements) {
        return;
    }
    let element = element_table[e];
    let corners = element.nodes;
    let nodal = array(
        pressures[corners[0]], pressures[corners[1]], pressures[corners[2]], pressures[corners[3]],
    );
    element_forces[e] = tet4_elastic_forces(
        element_displacements(corners),
        element.edge_inverse,
        element.rest_volume,
        element.material,
        sampled_element_pressure(nodal, dilations[e], element.stabilization),
    );
}

// Phase 4: each element's viscous forces at the bound velocities.
@compute @workgroup_size(64)
fn viscous_element_forces(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let e = item(id, groups, WORKGROUP);
    if (e >= constants.elements) {
        return;
    }
    let element = element_table[e];
    element_forces[e] = tet4_viscous_forces(
        element_displacements(element.nodes),
        element_velocities(element.nodes),
        element.edge_inverse,
        element.rest_volume,
        element.material,
    );
}

// Phase 5: each node's force, its elements' slots summed over its incidence
// list in order.
@compute @workgroup_size(64)
fn gather_forces(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    var sum = array(0.0, 0.0, 0.0);
    for (var slot = offsets[a]; slot < offsets[a + 1u]; slot++) {
        let entry = entries[slot];
        let e = entry / 4u;
        let first = 3u * (entry % 4u);
        sum = array(
            sum[0] + element_forces[e][first],
            sum[1] + element_forces[e][first + 1u],
            sum[2] + element_forces[e][first + 2u],
        );
    }
    node_forces[a] = sum;
}

// ---- Phases 6–8 ----

// Phase 6: each surface node's contact, with the step's terms for its row:
// the force, its moment about the rest centroid, and the normal force.
@compute @workgroup_size(64)
fn contact(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) local: u32,
) {
    keep_order();
    let i = item(id, groups, WORKGROUP);
    var coarse = false;
    if (i < constants.surface) {
        let a = surface_nodes[i];
        let node = node_table[a];
        let start = step_values.start;
        let end = step_values.end;
        let dt = step_values.dt;
        let damping = step_values.damping;
        let displacement = displacements[a];
        let sample = sample_obstacle(pose_to_body(start, vec3_add(node.rest, displacement))).sample;
        let stiffness = kinematic_stiffness(node.mass, node.inverse_mass, damping, dt);
        // Where the node lands without contact, its constraints applied.
        let velocity = free_part(node, advance_velocity(
            velocities[a],
            vec3_add(elastic_forces[a], viscous_forces[a]),
            node.inverse_mass,
            damping,
            dt,
        ));
        let point = vec3_add(node.rest, free_part(node, advance_displacement(displacement, velocity, dt)));
        let predicted = sample_obstacle(pose_to_body(end, point));
        // The normal where the node is now, carried into the step's end frame.
        let normal = pose_unrotate(end, pose_rotate(start, sample.normal));
        let response = kinematic_contact(
            end,
            point,
            SdfSample(predicted.sample.distance, normal),
            anchors[i],
            stiffness,
            constants.friction,
            node.constraints,
        );
        contacts[i] = Contact(response.force, response.normal_force, response.friction);
        anchors[i] = response.anchor;
        var deepest = maxima[i];
        deepest[0] = max(deepest[0], max(-sample.distance, 0.0));
        // A node the law cannot move, held whole, takes no correction, so its
        // depth is not read.
        if (stiffness > 0.0) {
            deepest[1] = max(deepest[1], max(-predicted.sample.distance, 0.0));
        }
        maxima[i] = deepest;
        coarse = response.normal_force > 0.0 && !predicted.fine;
        let moment = vec3_cross(vec3_add(node.arm, displacement), response.force);
        let first = 7u * i;
        terms[first] = response.force[0];
        terms[first + 1u] = response.force[1];
        terms[first + 2u] = response.force[2];
        terms[first + 3u] = moment[0];
        terms[first + 4u] = moment[1];
        terms[first + 5u] = moment[2];
        terms[first + 6u] = response.normal_force;
    }
    add_count(COARSE, coarse, local);
}

// Phase 7: the central-difference update.
@compute @workgroup_size(64)
fn integrate(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    let node = node_table[a];
    let velocity = velocities[a];
    previous_velocities[a] = velocity;
    let force = vec3_add(vec3_add(elastic_forces[a], viscous_forces[a]), contact_force(node));
    let advanced = advance_velocity(velocity, force, node.inverse_mass, step_values.damping, step_values.dt);
    velocities[a] = advanced;
    displacements[a] = advance_displacement(displacements[a], advanced, step_values.dt);
}

// Phase 8: each node's constraints, and its terms for the step's boundary
// row: the contact work, and the energy damping removed.
@compute @workgroup_size(64)
fn boundary_conditions(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    let node = node_table[a];
    let velocity = free_part(node, velocities[a]);
    velocities[a] = velocity;
    displacements[a] = free_part(node, displacements[a]);
    let previous = previous_velocities[a];
    let viscous = step_work(viscous_forces[a], previous, velocity, step_values.dt);
    terms[2u * a] = step_work(contact_force(node), previous, velocity, step_values.dt);
    terms[2u * a + 1u] =
        damping_loss(node.mass, step_values.damping, previous, velocity, step_values.dt) - viscous;
}

// Each node's displacement, and each surface node's normal and friction
// force, added to the window sums.
@compute @workgroup_size(64)
fn accumulate(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    let displacement = displacements[a];
    for (var d = 0u; d < 3u; d++) {
        window[3u * a + d] = window[3u * a + d] + displacement[d];
    }
    let i = node_table[a].surface;
    if (i != NONE) {
        let touching = contacts[i];
        let normal = 3u * constants.nodes + i;
        window[normal] = window[normal] + touching.normal_force;
        let friction = 3u * constants.nodes + constants.surface + 3u * i;
        for (var d = 0u; d < 3u; d++) {
            window[friction + d] = window[friction + d] + touching.friction[d];
        }
    }
}

// ---- Reductions ----

var<workgroup> tree: array<f32, 1792>;

// The reduction's operation on two values. On Metal `max` drops a NaN, as
// Rust's does (recon §17b); on other backends that is not measured.
fn combine(a: f32, b: f32) -> f32 {
    if (reduction.operation == SUM) {
        return a + b;
    }
    return max(a, b);
}

// Reduce each component's `TREE` values in `tree` by a fixed tree, into the
// first of them. Called from uniform control flow by every invocation.
fn reduce_tree(local: u32) {
    workgroupBarrier();
    for (var stride = TREE / 2u; stride > 0u; stride = stride / 2u) {
        if (local < stride) {
            for (var c = 0u; c < reduction.components; c++) {
                let at = c * TREE + local;
                tree[at] = combine(tree[at], tree[at + stride]);
            }
        }
        workgroupBarrier();
    }
}

// Each workgroup's `TREE` items reduced into one partial per component. A
// dispatch in two dimensions runs more workgroups than there are partials;
// those write nothing.
@compute @workgroup_size(256)
fn reduce_partials(
    @builtin(workgroup_id) group: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) local: u32,
) {
    keep_order();
    let block = group.x + group.y * groups.x;
    let blocks = (reduction.items + TREE - 1u) / TREE;
    let i = block * TREE + local;
    let components = reduction.components;
    for (var c = 0u; c < components; c++) {
        var value = 0.0;
        if (i < reduction.items) {
            value = terms[i * components + c];
        }
        tree[c * TREE + local] = value;
    }
    reduce_tree(local);
    if (local == 0u && block < blocks) {
        for (var c = 0u; c < components; c++) {
            partials[block * components + c] = tree[c * TREE];
        }
    }
}

// The partials reduced into the reduction's row, one workgroup: each
// invocation takes every `TREE`th partial in order, then a fixed tree.
@compute @workgroup_size(256)
fn reduce_row(@builtin(local_invocation_index) local: u32) {
    keep_order();
    let components = reduction.components;
    let blocks = (reduction.items + TREE - 1u) / TREE;
    for (var c = 0u; c < components; c++) {
        var value = 0.0;
        for (var b = local; b < blocks; b += TREE) {
            value = combine(value, partials[b * components + c]);
        }
        tree[c * TREE + local] = value;
    }
    reduce_tree(local);
    if (local == 0u) {
        var row = reduction.row;
        if (row == CONTACT_ROW) {
            row = step_values.contact_row;
        } else if (row == BOUNDARY_ROW) {
            row = step_values.boundary_row;
        }
        for (var c = 0u; c < components; c++) {
            log_rows[row * components + c] = tree[c * TREE];
        }
    }
}

// ---- The internal energy, read ----

// Each element's energy: its μ terms and the part of its λ term taken at its
// own volume.
@compute @workgroup_size(64)
fn element_energies(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let e = item(id, groups, WORKGROUP);
    if (e >= constants.elements) {
        return;
    }
    let element = element_table[e];
    terms[e] = tet4_energy_mu_terms(
        element_displacements(element.nodes),
        element.edge_inverse,
        element.rest_volume,
        element.material,
    ) + element.rest_volume * energy_density_lambda_term(dilations[e], element.stabilization);
}

// Each node's kinetic energy, the same again if it was in contact at the
// last step, and the averaged part of its λ term.
@compute @workgroup_size(64)
fn node_energies(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    let node = node_table[a];
    let kinetic = kinetic_energy(node.mass, velocities[a]);
    var in_contact = 0.0;
    if (node.surface != NONE && contacts[node.surface].normal_force > 0.0) {
        in_contact = kinetic;
    }
    terms[3u * a] = kinetic;
    terms[3u * a + 1u] = in_contact;
    terms[3u * a + 2u] = node.rest_volume * energy_density_lambda_term(
        nodal_dilation(volume_changes[a], node.rest_volume),
        node.lambda,
    );
}

// ---- The power iteration ----
//
// `top_mode_and_vector` in `src/cpu/executor.rs`, one dispatch a loop body
// step. A break sets `STOPPED`, which idles the later iterations. Only
// `estimate_start`, which does not read it, and the single-invocation
// `estimate_scale` write it, so no dispatch reads it while it changes.

// The start vector, and the scalars cleared.
@compute @workgroup_size(64)
fn estimate_start(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a == 0u) {
        scalars[STOPPED] = 0.0;
        scalars[LARGEST] = 0.0;
        // Not zero, so the first iteration does not read a break.
        scalars[SIZE] = 1.0;
        scalars[SCALE] = 0.0;
        scalars[TAKEN] = 0.0;
    }
    if (a >= constants.nodes) {
        return;
    }
    let v = start_vector[a];
    vector[a] = v;
    measured[a] = v;
    magnitudes[a] = largest_component(v);
}

// An iteration's start: stop at a vector of zeros, or after the last
// iteration's scaled next vector was zero; else the finite difference's step.
@compute @workgroup_size(1)
fn estimate_scale() {
    keep_order();
    if (scalars[STOPPED] != 0.0) {
        return;
    }
    let largest = scalars[LARGEST];
    if (largest == 0.0 || scalars[SIZE] == 0.0) {
        scalars[STOPPED] = 1.0;
        return;
    }
    scalars[SCALE] = step_values.perturbation / largest;
}

// The displacements moved along the vector.
@compute @workgroup_size(64)
fn estimate_shift(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes || scalars[STOPPED] != 0.0) {
        return;
    }
    shifted[a] = vec3_add(displacements[a], vec3_scale(vector[a], scalars[SCALE]));
}

// `K v`, the finite difference in the free directions, and the stiffness
// quotient's terms `v · K v` and `m v · v`.
@compute @workgroup_size(64)
fn estimate_stiffness(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes || scalars[STOPPED] != 0.0) {
        return;
    }
    if (a == 0u) {
        scalars[TAKEN] = 1.0;
    }
    let node = node_table[a];
    let v = vector[a];
    let difference = vec3_sub(shifted_forces[a], base_forces[a]);
    let stiffness = free_part(node, vec3_scale(difference, -1.0 / scalars[SCALE]));
    next_vector[a] = stiffness;
    quotients[2u * a] = vec3_dot(v, stiffness);
    quotients[2u * a + 1u] = node.mass * vec3_dot(v, v);
}

// The next iterate `M⁻¹ (K + βC) v`, with `C v` minus the viscous forces at
// velocities `v`.
@compute @workgroup_size(64)
fn estimate_next(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes || scalars[STOPPED] != 0.0) {
        return;
    }
    let node = node_table[a];
    var iterate = next_vector[a];
    if (step_values.weight > 0.0) {
        let damping = free_part(node, vec3_scale(viscous_at[a], -1.0));
        iterate = vec3_add(iterate, vec3_scale(damping, step_values.weight));
    }
    let next = vec3_scale(iterate, node.inverse_mass);
    next_vector[a] = next;
    magnitudes[a] = largest_component(next);
}

// The vector the quotients were taken of kept, and the next iterate scaled
// to a largest component of 1; left as it is when that is zero, which the
// next iteration's start reads as a break.
@compute @workgroup_size(64)
fn estimate_advance(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes || scalars[STOPPED] != 0.0) {
        return;
    }
    measured[a] = vector[a];
    let size = scalars[SIZE];
    if (size != 0.0) {
        let scaled = vec3_scale(next_vector[a], 1.0 / size);
        vector[a] = scaled;
        magnitudes[a] = largest_component(scaled);
    }
}

// The damping quotient's terms for the vector returned: `v · C v` and
// `m v · v`.
@compute @workgroup_size(64)
fn estimate_damping(
    @builtin(global_invocation_id) id: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
) {
    keep_order();
    let a = item(id, groups, WORKGROUP);
    if (a >= constants.nodes) {
        return;
    }
    let node = node_table[a];
    let v = measured[a];
    dampings[2u * a] = vec3_dot(v, free_part(node, vec3_scale(viscous_at[a], -1.0)));
    dampings[2u * a + 1u] = node.mass * vec3_dot(v, v);
}
