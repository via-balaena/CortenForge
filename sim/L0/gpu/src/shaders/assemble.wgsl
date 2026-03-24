// assemble.wgsl — Constraint assembly for the GPU physics pipeline.
//
// Reads contacts from the collision pipeline (PipelineContact buffer) and
// produces constraint rows for the Newton solver: Jacobians (efc_J),
// diagonal regularisation (efc_D), and reference acceleration (efc_aref).
//
// One thread per contact, @workgroup_size(256). Each thread:
//   1. Reads one PipelineContact from the raw f32 buffer.
//   2. Computes an orthonormal tangent frame from the contact normal.
//   3. Emits pyramidal friction rows (condim-dependent row count).
//   4. Computes impedance-based regularisation and KBIP reference accel.
//
// Friction model: pyramidal (MJX-compatible).
//   condim=1 → 1 row  (frictionless normal)
//   condim=3 → 4 rows (2 tangents x 2 facets)
//   condim=4 → 6 rows (2 tangents + 1 torsional x 2 facets)
//
// Jacobian layout: FREE JOINTS ONLY (6 DOFs per body).
//   DOFs 0..3 = translational (world-frame linear velocity)
//   DOFs 3..6 = rotational    (world-frame angular velocity)

// ── Struct definitions ──────────────────────────────────────────────

struct AssemblyParams {
    nv: u32,
    max_contacts: u32,
    max_constraints: u32,
    nbody: u32,
    timestep: f32,
    impratio: f32,
    solref_timeconst: f32,
    solref_dampratio: f32,
    solimp_d0: f32,
    solimp_dwidth: f32,
    solimp_width: f32,
    solimp_midpoint: f32,
    solimp_power: f32,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
};

struct BodyModel {
    parent: u32,
    depth: u32,
    jnt_adr: u32,
    jnt_num: u32,
    dof_adr: u32,
    dof_num: u32,
    mocap_id: u32,
    _pad: u32,
    pos: vec4<f32>,
    quat: vec4<f32>,
    ipos: vec4<f32>,
    iquat: vec4<f32>,
    inertia: vec4<f32>,
};

struct GeomModel {
    body_id: u32,
    geom_type: u32,
    contype: u32,
    conaffinity: u32,
    pos: vec4<f32>,
    quat: vec4<f32>,
    size: vec4<f32>,
    friction: vec4<f32>,
    sdf_meta_idx: u32,
    condim: u32,
    _pad0: u32,
    _pad1: u32,
};

// NOTE: PipelineContact is NOT declared as a struct because WGSL does not
// support vec3 in storage buffer structs (alignment mismatch). Instead we
// read 12 raw f32 values per contact from the flat buffer. See binding
// group 2, binding 0.

// ── Bindings ────────────────────────────────────────────────────────

// Group 0: Assembly parameters (uniform)
@group(0) @binding(0) var<uniform> params: AssemblyParams;

// Group 1: Static model data (read-only)
@group(1) @binding(0) var<storage, read> geoms: array<GeomModel>;
@group(1) @binding(1) var<storage, read> bodies: array<BodyModel>;
@group(1) @binding(2) var<storage, read> body_invweight0: array<vec4<f32>>;

// Group 2: Collision pipeline outputs + runtime state (read-only)
//   contact_buf: flat f32 array, 12 floats per contact (48 bytes).
//     [0..3]  = point.xyz, depth
//     [4..7]  = normal.xyz, geom1 (bitcast<u32>)
//     [8..11] = friction.xyz, geom2 (bitcast<u32>)
@group(2) @binding(0) var<storage, read> contact_buf: array<f32>;
@group(2) @binding(1) var<storage, read_write> contact_count_buf: array<atomic<u32>>;
@group(2) @binding(2) var<storage, read> body_xpos: array<vec4<f32>>;
@group(2) @binding(3) var<storage, read> body_xquat: array<vec4<f32>>;
@group(2) @binding(4) var<storage, read> qvel: array<f32>;

// Group 3: Constraint outputs (read-write)
//   efc_J:  row-major Jacobian, shape [max_constraints, nv]
//   efc_D:  diagonal regularisation, shape [max_constraints]
//   efc_aref: reference acceleration, shape [max_constraints]
//   constraint_count: atomic counter for allocated rows
@group(3) @binding(0) var<storage, read_write> efc_J: array<f32>;
@group(3) @binding(1) var<storage, read_write> efc_D: array<f32>;
@group(3) @binding(2) var<storage, read_write> efc_aref: array<f32>;
@group(3) @binding(3) var<storage, read_write> constraint_count: array<atomic<u32>>;

// ── Helper: quaternion to rotation matrix ───────────────────────────

/// Convert unit quaternion (x,y,z,w) to 3x3 rotation matrix (column-major).
fn quat_to_mat3(q: vec4<f32>) -> mat3x3<f32> {
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let w = q.w;

    let x2 = x + x;
    let y2 = y + y;
    let z2 = z + z;

    let xx = x * x2;
    let xy = x * y2;
    let xz = x * z2;
    let yy = y * y2;
    let yz = y * z2;
    let zz = z * z2;
    let wx = w * x2;
    let wy = w * y2;
    let wz = w * z2;

    return mat3x3<f32>(
        vec3<f32>(1.0 - (yy + zz), xy + wz, xz - wy),   // column 0
        vec3<f32>(xy - wz, 1.0 - (xx + zz), yz + wx),    // column 1
        vec3<f32>(xz + wy, yz - wx, 1.0 - (xx + yy)),    // column 2
    );
}

// ── Helper: tangent frame from normal ───────────────────────────────

/// Compute orthonormal tangent vectors from a contact normal.
/// Returns [t1, t2] such that {normal, t1, t2} form a right-handed frame.
fn compute_tangent_frame(normal: vec3<f32>) -> array<vec3<f32>, 2> {
    // Choose reference axis that is not nearly parallel to the normal.
    let ref_axis = select(vec3<f32>(1.0, 0.0, 0.0),
                          vec3<f32>(0.0, 1.0, 0.0),
                          abs(normal.x) > 0.9);
    let t1 = normalize(cross(normal, ref_axis));
    let t2 = cross(normal, t1);
    return array<vec3<f32>, 2>(t1, t2);
}

// ── Helper: lever arm stabilisation ─────────────────────────────────

/// If the component of r perpendicular to the contact normal is
/// sub-physically small, project r onto the normal axis to avoid
/// numerical noise in the torque arm.
fn stabilize_lever_arm(r: vec3<f32>, normal: vec3<f32>) -> vec3<f32> {
    let r_len = length(r);
    let r_along_n = dot(r, normal) * normal;
    let r_perp = r - r_along_n;
    let perp_len = length(r_perp);

    let threshold = 1e-6 * max(r_len, 1e-10);
    if perp_len < threshold {
        return r_along_n;
    }
    return r;
}

// ── Helper: read one PipelineContact from raw f32 buffer ────────────

struct ContactData {
    point: vec3<f32>,
    depth: f32,
    normal: vec3<f32>,
    geom1: u32,
    friction: vec3<f32>,
    geom2: u32,
};

fn read_contact(ci: u32) -> ContactData {
    let base = ci * 12u;
    var c: ContactData;
    c.point   = vec3<f32>(contact_buf[base + 0u],
                          contact_buf[base + 1u],
                          contact_buf[base + 2u]);
    c.depth   = contact_buf[base + 3u];
    c.normal  = vec3<f32>(contact_buf[base + 4u],
                          contact_buf[base + 5u],
                          contact_buf[base + 6u]);
    c.geom1   = bitcast<u32>(contact_buf[base + 7u]);
    c.friction = vec3<f32>(contact_buf[base + 8u],
                           contact_buf[base + 9u],
                           contact_buf[base + 10u]);
    c.geom2   = bitcast<u32>(contact_buf[base + 11u]);
    return c;
}

// ── Helper: zero a Jacobian row ─────────────────────────────────────

fn zero_row(row: u32) {
    let base = row * params.nv;
    for (var k = 0u; k < params.nv; k++) {
        efc_J[base + k] = 0.0;
    }
}

// ── Helper: write Jacobian entries for one body ─────────────────────

/// Write Jacobian entries for body B into a given row.
///
/// `direction` is the force direction (normal, facet, or torsional).
/// `sign` is -1 for body1 (reaction), +1 for body2 (action).
/// `row` is the constraint row index.
/// `body_id` is the body index.
/// `contact_pt` is the world-space contact point.
/// `normal` is the contact normal (used for torsional angular term).
/// `mu_torsional` is the torsional friction coefficient (0 if not torsional).
/// `torsional_sign` is +1 or -1 for the torsional facet direction.
/// `is_torsional` selects between pure-direction Jacobian (false) and
///   torsional Jacobian (true) which has a different angular term.
fn write_jacobian_body(
    row: u32,
    body_id: u32,
    sign: f32,
    direction: vec3<f32>,
    contact_pt: vec3<f32>,
    normal: vec3<f32>,
    mu_torsional: f32,
    torsional_sign: f32,
    is_torsional: bool,
) {
    // World body (id=0) has no DOFs — skip.
    if body_id == 0u { return; }

    let dof_adr = bodies[body_id].dof_adr;
    let r_raw = contact_pt - body_xpos[body_id].xyz;
    let r = stabilize_lever_arm(r_raw, normal);
    let rot = quat_to_mat3(body_xquat[body_id]);

    let base = row * params.nv;

    if is_torsional {
        // Torsional facet: translational part is the normal direction,
        // angular part includes the torsional friction term.
        efc_J[base + dof_adr + 0u] += sign * normal.x;
        efc_J[base + dof_adr + 1u] += sign * normal.y;
        efc_J[base + dof_adr + 2u] += sign * normal.z;

        // Angular DOFs: dot(normal, cross(omega_i, r)) +/- mu_t * dot(normal, omega_i)
        for (var i = 0u; i < 3u; i++) {
            let omega_i = rot[i]; // column i of rotation matrix
            let torque_arm = dot(normal, cross(omega_i, r));
            let torsion_term = torsional_sign * mu_torsional * dot(normal, omega_i);
            efc_J[base + dof_adr + 3u + i] += sign * (torque_arm + torsion_term);
        }
    } else {
        // Sliding or frictionless: Jacobian in the given direction.
        efc_J[base + dof_adr + 0u] += sign * direction.x;
        efc_J[base + dof_adr + 1u] += sign * direction.y;
        efc_J[base + dof_adr + 2u] += sign * direction.z;

        // Angular DOFs: dot(direction, cross(omega_i, r))
        for (var i = 0u; i < 3u; i++) {
            let omega_i = rot[i]; // column i of rotation matrix
            efc_J[base + dof_adr + 3u + i] += sign * dot(direction, cross(omega_i, r));
        }
    }
}

// ── Helper: impedance sigmoid ───────────────────────────────────────

/// Compute impedance value from violation depth using the solimp sigmoid.
fn compute_impedance(violation: f32) -> f32 {
    let d0 = params.solimp_d0;
    let dwidth = params.solimp_dwidth;
    let width = params.solimp_width;
    let midpoint = params.solimp_midpoint;
    let power = params.solimp_power;

    // Degenerate width: return midpoint of impedance range.
    if width <= 1e-10 {
        return 0.5 * (d0 + dwidth);
    }

    let x = min(violation / width, 1.0);

    if x >= 1.0 {
        return dwidth;
    }
    if x <= 0.0 {
        return d0;
    }

    // Sigmoid interpolation between d0 and dwidth.
    var y: f32;
    if abs(power - 1.0) < 1e-12 {
        // Linear case.
        y = x;
    } else if x <= midpoint {
        // Left branch: y = x^p / midpoint^(p-1)
        y = pow(x, power) / pow(midpoint, power - 1.0);
    } else {
        // Right branch: y = 1 - (1-x)^p / (1-midpoint)^(p-1)
        y = 1.0 - pow(1.0 - x, power) / pow(1.0 - midpoint, power - 1.0);
    }

    let imp = d0 + y * (dwidth - d0);
    return clamp(imp, 0.0001, 0.9999);
}

// ── Helper: compute D and aref for one row ──────────────────────────

/// Compute regularisation (D) and reference acceleration (aref) for a
/// single constraint row.
///
/// `row` — constraint row index.
/// `depth` — penetration depth (>= 0).
/// `body1` — body index for geom1.
/// `body2` — body index for geom2.
/// `is_pyramidal` — true for friction facet rows (scale bodyweight by 1+mu^2).
/// `mu` — friction coefficient for bodyweight scaling.
fn compute_D_aref(
    row: u32,
    depth: f32,
    body1: u32,
    body2: u32,
    is_pyramidal: bool,
    mu: f32,
) {
    let nv = params.nv;

    // ── Impedance from violation depth ──
    let violation = depth;
    let pos = -depth; // negative = penetration
    let imp = compute_impedance(violation);

    // ── diagApprox (bodyweight) ──
    // World body (id=0) has invweight0 = 0 implicitly (buffer must be
    // initialised with 0 for body 0).
    var bw = 0.0;
    if body1 != 0u {
        bw += body_invweight0[body1].x;
    }
    if body2 != 0u {
        bw += body_invweight0[body2].x;
    }
    if is_pyramidal {
        bw *= (1.0 + mu * mu);
    }
    let diag = max(bw, 1e-15);

    // ── Regularisation ──
    let R = max((1.0 - imp) / imp * diag, 1e-15);
    let D_val = 1.0 / R;

    // ── KBIP (stiffness + damping from solref) ──
    let dmax = clamp(params.solimp_dwidth, 0.0001, 0.9999);
    let timeconst = max(params.solref_timeconst, 2.0 * params.timestep);
    let dampratio = params.solref_dampratio;
    let K = 1.0 / max(dmax * dmax * timeconst * timeconst
                       * dampratio * dampratio, 1e-15);
    let B = 2.0 / max(dmax * timeconst, 1e-15);

    // ── Velocity: vel = J_row . qvel ──
    let base = row * nv;
    var vel = 0.0;
    for (var k = 0u; k < nv; k++) {
        vel += efc_J[base + k] * qvel[k];
    }

    // ── Reference acceleration ──
    let aref_val = -B * vel - K * imp * pos;

    // ── Store ──
    efc_D[row] = D_val;
    efc_aref[row] = aref_val;
}

// ── Main entry point ────────────────────────────────────────────────

@compute @workgroup_size(256)
fn assemble_constraints(@builtin(global_invocation_id) gid: vec3<u32>) {
    let ci = gid.x;

    // 1. Bounds check: only process existing contacts.
    let n_contacts = atomicLoad(&contact_count_buf[0]);
    if ci >= n_contacts {
        return;
    }

    // 2. Read contact from raw f32 buffer.
    let contact = read_contact(ci);

    // 3. Look up geom properties.
    let g1 = geoms[contact.geom1];
    let g2 = geoms[contact.geom2];
    let condim = min(g1.condim, g2.condim);

    // MuJoCo convention: body2 is the "pushed" body (gets +1 sign).
    // Normal points FROM body1 TO body2.
    //
    // The GPU SDF-plane shader outputs normal = plane_normal (upward for
    // ground). For SDF-plane: geom1 = SDF (free body), geom2 = plane (world).
    // The normal (0,0,1) already points FROM the contact surface TOWARD the
    // plane, which is FROM the free body TO the world — same direction as
    // FROM lower-index body TO higher-index body.
    //
    // We swap body indices so body1 = lower (world), body2 = higher (free
    // body). The normal stays unchanged because it already points from
    // the lower-index body toward the higher-index body.
    var normal = contact.normal;
    var body1 = g1.body_id;
    var body2 = g2.body_id;
    if body1 > body2 {
        let tmp = body1;
        body1 = body2;
        body2 = tmp;
        // Normal already points from the lower body toward the higher body
        // (plane normal = upward = from world toward free body). No negate.
    }

    // 4. Compute tangent frame.
    let tangents = compute_tangent_frame(normal);
    let t1 = tangents[0];
    let t2 = tangents[1];

    // 5. Determine row count from condim.
    var n_rows: u32;
    switch condim {
        case 1u: { n_rows = 1u; } // frictionless
        case 3u: { n_rows = 4u; } // 2 tangents x 2 facets
        case 4u: { n_rows = 6u; } // (2 tangents + 1 torsional) x 2 facets
        default: { n_rows = 4u; } // fallback to condim=3
    }

    // 6. Atomically allocate constraint rows.
    let row_start = atomicAdd(&constraint_count[0], n_rows);
    if row_start + n_rows > params.max_constraints {
        return;
    }

    // 7. Friction coefficients from the contact.
    let mu_slide = contact.friction.x;
    let mu_torsion = contact.friction.y;

    // ── Emit rows ───────────────────────────────────────────────────

    if condim == 1u {
        // ── FRICTIONLESS: single normal row ──
        let row = row_start;
        zero_row(row);

        // Jacobian: direction = normal
        write_jacobian_body(row, body1, -1.0, normal, contact.point,
                            normal, 0.0, 0.0, false);
        write_jacobian_body(row, body2,  1.0, normal, contact.point,
                            normal, 0.0, 0.0, false);

        compute_D_aref(row, contact.depth, body1, body2, false, 0.0);

    } else {
        // ── PYRAMIDAL SLIDING FACETS (condim >= 3) ──
        // 2 tangent directions x 2 facet signs = 4 rows.
        var row_idx = 0u;
        for (var td = 0u; td < 2u; td++) {
            let tangent_d = select(t1, t2, td == 1u);
            for (var facet = 0u; facet < 2u; facet++) {
                let facet_sign = select(1.0, -1.0, facet == 1u);
                let facet_dir = normal + facet_sign * mu_slide * tangent_d;
                let row = row_start + row_idx;

                zero_row(row);

                write_jacobian_body(row, body1, -1.0, facet_dir, contact.point,
                                    normal, 0.0, 0.0, false);
                write_jacobian_body(row, body2,  1.0, facet_dir, contact.point,
                                    normal, 0.0, 0.0, false);

                compute_D_aref(row, contact.depth, body1, body2, true, mu_slide);

                row_idx += 1u;
            }
        }

        // ── TORSIONAL FACETS (condim >= 4) ──
        if condim >= 4u {
            for (var facet = 0u; facet < 2u; facet++) {
                let facet_sign = select(1.0, -1.0, facet == 1u);
                let row = row_start + row_idx;

                zero_row(row);

                // Torsional: translational Jacobian is normal, angular
                // Jacobian includes +/- mu_t * dot(normal, omega_i).
                write_jacobian_body(row, body1, -1.0, normal, contact.point,
                                    normal, mu_torsion, facet_sign, true);
                write_jacobian_body(row, body2,  1.0, normal, contact.point,
                                    normal, mu_torsion, facet_sign, true);

                compute_D_aref(row, contact.depth, body1, body2, true, mu_torsion);

                row_idx += 1u;
            }
        }
    }
}
