//! Passive forces: springs, dampers, fluid drag, gravity compensation.
//!
//! Computes qfrc_passive from joint springs/dampers, tendon springs/dampers,
//! flex bending, fluid forces, and gravity compensation. Corresponds to
//! MuJoCo's `engine_passive.c`.

use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::object_velocity_local;
use crate::dynamics::rne::mj_gravcomp;
use crate::integrate::implicit::tendon_all_dofs_sleeping;
use crate::jacobian::mj_apply_ft;
use crate::joint_visitor::{JointContext, JointVisitor};
use crate::tendon::apply_tendon_force;
use crate::tendon::subquat;
use crate::types::{
    DISABLE_DAMPER, DISABLE_SPRING, Data, ENABLE_SLEEP, FlexBendingType, GeomType, Integrator,
    MjJointType, Model, SleepState,
};
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

/// Euclidean norm of a 3-element slice.
#[inline]
pub fn norm3(v: &[f64]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Compute semi-axes for a geom type, matching MuJoCo's `mju_geomSemiAxes`.
pub fn fluid_geom_semi_axes(geom_type: GeomType, size: &Vector3<f64>) -> [f64; 3] {
    match geom_type {
        GeomType::Sphere => [size.x, size.x, size.x],
        GeomType::Capsule => [size.x, size.x, size.y + size.x],
        GeomType::Cylinder => [size.x, size.x, size.y],
        _ => [size.x, size.y, size.z], // Ellipsoid, Box, Mesh
    }
}

/// Per-axis moment for angular drag: `(8/15)π · s[axis] · max(other_two)⁴`.
/// Matches MuJoCo's `mji_ellipsoid_max_moment(size, dir)`.
#[inline]
pub fn ellipsoid_moment(s: &[f64; 3], axis: usize) -> f64 {
    let d1 = s[(axis + 1) % 3];
    let d2 = s[(axis + 2) % 3];
    (8.0 / 15.0) * std::f64::consts::PI * s[axis] * d1.max(d2).powi(4)
}

/// Rotate a 6D spatial vector (local frame) to world frame.
#[inline]
fn rotate_spatial_to_world(xmat: &Matrix3<f64>, lfrc: &[f64; 6]) -> [f64; 6] {
    let torque = xmat * Vector3::new(lfrc[0], lfrc[1], lfrc[2]);
    let force = xmat * Vector3::new(lfrc[3], lfrc[4], lfrc[5]);
    [torque.x, torque.y, torque.z, force.x, force.y, force.z]
}

/// Cross product of two 3-vectors stored as slices.
#[inline]
fn cross3(a: &[f64], b: &[f64]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Accumulate cross product: `out += a × b`.
#[inline]
fn add_cross3(out: &mut [f64], a: &[f64; 3], b: &[f64]) {
    out[0] += a[1] * b[2] - a[2] * b[1];
    out[1] += a[2] * b[0] - a[0] * b[2];
    out[2] += a[0] * b[1] - a[1] * b[0];
}

/// Inertia-box fluid model (legacy, body-level).
/// Called for bodies where no child geom has `geom_fluid[0] > 0`.
fn mj_inertia_box_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let mass = model.body_mass[body_id];
    let inertia = model.body_inertia[body_id];

    // 1. Equivalent box dimensions (full side lengths)
    let bx = ((inertia.y + inertia.z - inertia.x).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let by = ((inertia.x + inertia.z - inertia.y).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let bz = ((inertia.x + inertia.y - inertia.z).max(MJ_MINVAL) / mass * 6.0).sqrt();

    // 2. Local 6D velocity at body CoM in inertia frame
    let mut lvel = object_velocity_local(
        model,
        data,
        body_id,
        &data.xipos[body_id],
        &data.ximat[body_id],
    );

    // 3. Subtract wind (translational only, rotate to inertia frame)
    let wind_local = data.ximat[body_id].transpose() * model.wind;
    lvel[3] -= wind_local.x;
    lvel[4] -= wind_local.y;
    lvel[5] -= wind_local.z;

    // 4. Compute local forces
    let mut lfrc = [0.0f64; 6];

    // Viscous resistance (assignment, matching MuJoCo's mji_scl3)
    if beta > 0.0 {
        let diam = (bx + by + bz) / 3.0;
        let d3 = diam * diam * diam;
        let pi = std::f64::consts::PI;
        for i in 0..3 {
            lfrc[i] = -pi * d3 * beta * lvel[i];
        }
        for i in 0..3 {
            lfrc[3 + i] = -3.0 * pi * diam * beta * lvel[3 + i];
        }
    }

    // Quadratic drag (density, subtracted from viscous result)
    if rho > 0.0 {
        lfrc[3] -= 0.5 * rho * by * bz * lvel[3].abs() * lvel[3];
        lfrc[4] -= 0.5 * rho * bx * bz * lvel[4].abs() * lvel[4];
        lfrc[5] -= 0.5 * rho * bx * by * lvel[5].abs() * lvel[5];

        lfrc[0] -= rho * bx * (by.powi(4) + bz.powi(4)) / 64.0 * lvel[0].abs() * lvel[0];
        lfrc[1] -= rho * by * (bx.powi(4) + bz.powi(4)) / 64.0 * lvel[1].abs() * lvel[1];
        lfrc[2] -= rho * bz * (bx.powi(4) + by.powi(4)) / 64.0 * lvel[2].abs() * lvel[2];
    }

    // 5. Rotate to world frame and apply at body CoM
    let bfrc = rotate_spatial_to_world(&data.ximat[body_id], &lfrc);
    mj_apply_ft(
        model,
        &data.xpos,
        &data.xquat,
        &Vector3::new(bfrc[3], bfrc[4], bfrc[5]),
        &Vector3::new(bfrc[0], bfrc[1], bfrc[2]),
        &data.xipos[body_id],
        body_id,
        &mut data.qfrc_fluid,
    );
}

/// Ellipsoid fluid model (advanced, per-geom).
/// Called for bodies where any child geom has `geom_fluid[0] > 0`.
#[allow(clippy::similar_names)] // d_min/d_mid/d_max are standard notation for semi-axis ordering
fn mj_ellipsoid_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let pi = std::f64::consts::PI;

    let geom_adr = model.body_geom_adr[body_id];
    let geom_num = model.body_geom_num[body_id];

    for gid in geom_adr..geom_adr + geom_num {
        let fluid = &model.geom_fluid[gid];
        let interaction_coef = fluid[0];
        if interaction_coef == 0.0 {
            continue;
        }

        // Unpack coefficients
        let (c_blunt, c_slender, c_ang) = (fluid[1], fluid[2], fluid[3]);
        let (c_kutta, c_magnus) = (fluid[4], fluid[5]);
        let vmass = [fluid[6], fluid[7], fluid[8]];
        let vinertia = [fluid[9], fluid[10], fluid[11]];

        // Semi-axes
        let s = fluid_geom_semi_axes(model.geom_type[gid], &model.geom_size[gid]);

        // Local velocity at geom center in geom frame
        let geom_body = model.geom_body[gid];
        let mut lvel = object_velocity_local(
            model,
            data,
            geom_body,
            &data.geom_xpos[gid],
            &data.geom_xmat[gid],
        );
        let wind_local = data.geom_xmat[gid].transpose() * model.wind;
        lvel[3] -= wind_local.x;
        lvel[4] -= wind_local.y;
        lvel[5] -= wind_local.z;

        let w = [lvel[0], lvel[1], lvel[2]];
        let v = [lvel[3], lvel[4], lvel[5]];
        let mut lfrc = [0.0f64; 6];
        let speed = norm3(&v);

        // ── Component 1: Added mass (gyroscopic, accels=NULL) ──
        let pv = [
            rho * vmass[0] * v[0],
            rho * vmass[1] * v[1],
            rho * vmass[2] * v[2],
        ];
        let lv = [
            rho * vinertia[0] * w[0],
            rho * vinertia[1] * w[1],
            rho * vinertia[2] * w[2],
        ];
        add_cross3(&mut lfrc[3..6], &pv, &w); // force  += p_v × ω
        add_cross3(&mut lfrc[0..3], &pv, &v); // torque += p_v × v
        add_cross3(&mut lfrc[0..3], &lv, &w); // torque += L_v × ω

        // ── Component 2: Magnus lift ──
        let vol = (4.0 / 3.0) * pi * s[0] * s[1] * s[2];
        let mag = cross3(&w, &v);
        for i in 0..3 {
            lfrc[3 + i] += c_magnus * rho * vol * mag[i];
        }

        // ── Component 3: Kutta lift ──
        let norm_vec = [
            (s[1] * s[2]).powi(2) * v[0],
            (s[2] * s[0]).powi(2) * v[1],
            (s[0] * s[1]).powi(2) * v[2],
        ];
        let proj_denom = (s[1] * s[2]).powi(4) * v[0] * v[0]
            + (s[2] * s[0]).powi(4) * v[1] * v[1]
            + (s[0] * s[1]).powi(4) * v[2] * v[2];
        let proj_num = (s[1] * s[2] * v[0]).powi(2)
            + (s[2] * s[0] * v[1]).powi(2)
            + (s[0] * s[1] * v[2]).powi(2);
        let a_proj = pi * (proj_denom / proj_num.max(MJ_MINVAL)).sqrt();
        let cos_alpha = proj_num / (speed * proj_denom).max(MJ_MINVAL);

        let mut circ = cross3(&norm_vec, &v);
        let kutta_scale = c_kutta * rho * cos_alpha * a_proj;
        for val in &mut circ {
            *val *= kutta_scale;
        }
        let kf = cross3(&circ, &v);
        for i in 0..3 {
            lfrc[3 + i] += kf[i];
        }

        // ── Component 4: Combined linear drag ──
        let eq_d = (2.0 / 3.0) * (s[0] + s[1] + s[2]);
        let d_max = s[0].max(s[1]).max(s[2]);
        let d_min = s[0].min(s[1]).min(s[2]);
        let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
        let a_max = pi * d_max * d_mid;

        let drag_lin = beta * 3.0 * pi * eq_d
            + rho * speed * (a_proj * c_blunt + c_slender * (a_max - a_proj));
        for i in 0..3 {
            lfrc[3 + i] -= drag_lin * v[i];
        }

        // ── Component 5: Combined angular drag ──
        let i_max = (8.0 / 15.0) * pi * d_mid * d_max.powi(4);
        let ii = [
            ellipsoid_moment(&s, 0),
            ellipsoid_moment(&s, 1),
            ellipsoid_moment(&s, 2),
        ];
        let mom_visc = [
            w[0] * (c_ang * ii[0] + c_slender * (i_max - ii[0])),
            w[1] * (c_ang * ii[1] + c_slender * (i_max - ii[1])),
            w[2] * (c_ang * ii[2] + c_slender * (i_max - ii[2])),
        ];
        let drag_ang = beta * pi * eq_d.powi(3) + rho * norm3(&mom_visc);
        for i in 0..3 {
            lfrc[i] -= drag_ang * w[i];
        }

        // ── Scale by interaction coefficient and accumulate ──
        for val in &mut lfrc {
            *val *= interaction_coef;
        }
        let bfrc = rotate_spatial_to_world(&data.geom_xmat[gid], &lfrc);
        mj_apply_ft(
            model,
            &data.xpos,
            &data.xquat,
            &Vector3::new(bfrc[3], bfrc[4], bfrc[5]),
            &Vector3::new(bfrc[0], bfrc[1], bfrc[2]),
            &data.geom_xpos[gid],
            body_id,
            &mut data.qfrc_fluid,
        );
    }
}

/// Top-level fluid force dispatch. Returns `true` if any fluid forces were computed.
/// Matches MuJoCo's `mj_fluid()` in `engine_passive.c`.
fn mj_fluid(model: &Model, data: &mut Data) -> bool {
    if model.density == 0.0 && model.viscosity == 0.0 {
        return false;
    }

    // §40c: Sleep filtering — skip sleeping bodies (MuJoCo engine_passive.c pattern)
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;
    let nbody = if sleep_filter {
        data.nbody_awake
    } else {
        model.nbody
    };

    for idx in 0..nbody {
        let body_id = if sleep_filter {
            data.body_awake_ind[idx]
        } else {
            idx
        };

        // Mass guard — applies to both models (matches MuJoCo)
        if model.body_mass[body_id] < MJ_MINVAL {
            continue;
        }

        // Dispatch: does any child geom have geom_fluid[0] > 0?
        let geom_adr = model.body_geom_adr[body_id];
        let geom_num = model.body_geom_num[body_id];
        let use_ellipsoid =
            (geom_adr..geom_adr + geom_num).any(|gid| model.geom_fluid[gid][0] > 0.0);

        if use_ellipsoid {
            mj_ellipsoid_fluid(model, data, body_id);
        } else {
            mj_inertia_box_fluid(model, data, body_id);
        }
    }

    true
}

/// Compute passive forces (springs and dampers).
///
/// Implements MuJoCo's passive force model:
/// - **Spring**: τ = -stiffness * (q - springref)
/// - **Damper**: τ = -damping * qvel
///
/// Friction loss is handled entirely by solver constraint rows (§29),
/// not by passive forces.
///
/// # MuJoCo Semantics
///
/// The spring equilibrium is `jnt_springref`, NOT `qpos0`. These are distinct:
/// - `qpos0`: Initial joint position at model load (for `mj_resetData()`)
/// - `springref`: Spring equilibrium position (where spring force is zero)
///
/// A joint can start at q=0 but have a spring pulling toward springref=0.5.
///
/// # Implicit Integration Mode
///
/// When `model.integrator == Implicit`, spring and damper forces are handled
/// implicitly in `mj_fwd_acceleration_implicit()`. This function then only
/// initializes `qfrc_passive` to zero (no explicit passive contributions).
pub fn mj_fwd_passive(model: &Model, data: &mut Data) {
    // S4.7a: CortenForge nv == 0 guard — explicit early return for clarity.
    if model.nv == 0 {
        return;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nv_awake < model.nv;

    // Zero all passive force vectors (S4.7a).
    // When sleep is active, only zero awake DOFs (sleeping DOFs stay zero from init/reset).
    if sleep_filter {
        for &dof in &data.dof_awake_ind[..data.nv_awake] {
            data.qfrc_passive[dof] = 0.0;
            data.qfrc_spring[dof] = 0.0;
            data.qfrc_damper[dof] = 0.0;
            data.qfrc_fluid[dof] = 0.0;
            data.qfrc_gravcomp[dof] = 0.0;
        }
    } else {
        data.qfrc_passive.fill(0.0);
        data.qfrc_spring.fill(0.0);
        data.qfrc_damper.fill(0.0);
        data.qfrc_fluid.fill(0.0);
        data.qfrc_gravcomp.fill(0.0);
    }
    // qfrc_frictionloss is now populated post-solve from efc_force (§29).
    // No longer computed in passive forces.

    // S4.7a: When both spring AND damper are disabled, skip all sub-functions.
    // Force vectors are already zeroed above.
    if model.disableflags & DISABLE_SPRING != 0 && model.disableflags & DISABLE_DAMPER != 0 {
        return;
    }

    // S4.7b: Component-level spring/damper gates.
    let has_spring = model.disableflags & DISABLE_SPRING == 0;
    let has_damper = model.disableflags & DISABLE_DAMPER == 0;

    let implicit_mode = model.integrator == Integrator::ImplicitSpringDamper;
    {
        let mut visitor = PassiveForceVisitor {
            model,
            data,
            implicit_mode,
            sleep_enabled,
            has_spring,
            has_damper,
        };
        model.visit_joints(&mut visitor);
    }
    // visitor is dropped here, releasing the mutable borrow on data

    // Tendon passive forces: spring + damper (S4.7-prereq: separate arrays).
    for t in 0..model.ntendon {
        // §16.5a': Skip tendon if ALL target DOFs are sleeping
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let length = data.ten_length[t];
        let velocity = data.ten_velocity[t];
        let mut spring_force = 0.0;
        let mut damper_force = 0.0;

        // S4.7b + S6: Deadband spring — gated on DISABLE_SPRING.
        if has_spring {
            let k = model.tendon_stiffness[t];
            if k > 0.0 {
                let [lower, upper] = model.tendon_lengthspring[t];
                if length > upper {
                    spring_force += k * (upper - length);
                } else if length < lower {
                    spring_force += k * (lower - length);
                }
            }
        }

        // S4.7b: Damper — gated on DISABLE_DAMPER.
        if has_damper {
            let b = model.tendon_damping[t];
            if b > 0.0 {
                damper_force -= b * velocity;
            }
        }

        // Friction loss is now handled entirely by solver constraint rows (§29).
        // No tanh approximation in passive forces.

        data.ten_force[t] = spring_force + damper_force;

        // NOTE: In ImplicitSpringDamper mode, tendon spring/damper forces are
        // handled implicitly in mj_fwd_acceleration_implicit() via non-diagonal
        // K_tendon and D_tendon matrices (DT-35). ten_force[t] is always populated
        // for diagnostic purposes, but the explicit force application is
        // skipped to avoid double-counting, matching the joint spring/damper pattern.

        if !implicit_mode {
            // Map tendon spring force to joint forces via J^T.
            if spring_force != 0.0 {
                apply_tendon_force(
                    model,
                    &data.ten_J[t],
                    model.tendon_type[t],
                    t,
                    spring_force,
                    &mut data.qfrc_spring,
                );
            }
            // Map tendon damper force to joint forces via J^T.
            if damper_force != 0.0 {
                apply_tendon_force(
                    model,
                    &data.ten_J[t],
                    model.tendon_type[t],
                    t,
                    damper_force,
                    &mut data.qfrc_damper,
                );
            }
        }
    }

    // S4.7b: Flex vertex damping → qfrc_damper (gated on DISABLE_DAMPER).
    if has_damper {
        for i in 0..model.nflexvert {
            let dof_base = model.flexvert_dofadr[i];
            if dof_base == usize::MAX {
                continue; // Pinned vertex: no DOFs
            }
            let flex_id = model.flexvert_flexid[i];
            let damp = model.flex_damping[flex_id];
            if damp <= 0.0 {
                continue;
            }
            for k in 0..3 {
                data.qfrc_damper[dof_base + k] -= damp * data.qvel[dof_base + k];
            }
        }
    }

    // Flex edge passive spring-damper forces.
    // MuJoCo architecture: <edge stiffness="..." damping="..."/> drives passive
    // forces (engine_passive.c), separate from constraint-based edge enforcement
    // (mjEQ_FLEX in engine_core_constraint.c which uses eq_solref/eq_solimp).
    // Note: MuJoCo docs say <edge stiffness> is "Only for 1D flex" (cables).
    // For 2D/3D, elasticity comes from FEM via <elasticity>. The code applies
    // to all dims (matching MuJoCo's runtime behavior), but users should only
    // set nonzero stiffness for dim=1 flex bodies.
    for e in 0..model.nflexedge {
        let flex_id = model.flexedge_flexid[e];
        let stiffness = model.flex_edgestiffness[flex_id];
        let damping = model.flex_edgedamping[flex_id];

        if stiffness == 0.0 && damping == 0.0 {
            continue;
        }

        // §42A-ii: Skip rigid edges via pre-computed boolean lookup.
        if model.flexedge_rigid[e] {
            continue;
        }

        // §42A-iii: Read pre-computed edge length and velocity from Data fields.
        let dist = data.flexedge_length[e];
        if dist < 1e-10 {
            continue;
        }

        let rest_len = model.flexedge_length0[e];

        // S4.7b: Spring force gated on DISABLE_SPRING.
        let frc_spring = if has_spring {
            stiffness * (rest_len - dist)
        } else {
            0.0
        };

        // S4.7b: Damping force gated on DISABLE_DAMPER.
        let edge_velocity = data.flexedge_velocity[e];
        let frc_damper = if has_damper {
            -damping * edge_velocity
        } else {
            0.0
        };

        // §42A-i: J^T * force — unified sparse loop replacing inline ±direction.
        let rowadr = model.flexedge_J_rowadr[e];
        let rownnz = model.flexedge_J_rownnz[e];
        for j in 0..rownnz {
            let col = model.flexedge_J_colind[rowadr + j];
            let jval = data.flexedge_J[rowadr + j];
            if frc_spring != 0.0 {
                data.qfrc_spring[col] += jval * frc_spring;
            }
            if frc_damper != 0.0 {
                data.qfrc_damper[col] += jval * frc_damper;
            }
        }
    }

    // §42B S6+S7: Flex bending passive forces — dispatched by FlexBendingType.
    // MuJoCo computes bending as passive forces in engine_passive.c:192–268.
    // Cotangent (default): precomputed 4x4 B matrix × position (MuJoCo-conformant).
    // Bridson (extension): dihedral angle spring-damper with per-vertex stability clamp.
    for f in 0..model.nflex {
        let dim = model.flex_dim[f];
        if dim != 2 || model.flex_rigid[f] {
            continue;
        }

        match model.flex_bending_type[f] {
            FlexBendingType::Cotangent => {
                // §42B S6: Cotangent Laplacian bending (MuJoCo-conformant).
                // Runtime: B * xpos (spring) + B * vel (damper) + b[16] * frc (curved ref).
                // MuJoCo ref: engine_passive.c:206–268.
                let edge_adr = model.flex_edgeadr[f];
                let edge_num = model.flex_edgenum[f];
                let damping_coeff = model.flex_damping[f];

                for local_e in 0..edge_num {
                    let e = edge_adr + local_e;
                    let flap = model.flexedge_flap[e];
                    if flap[1] == -1 {
                        continue; // boundary edge
                    }

                    // Diamond stencil vertices: [edge0, edge1, opp_tri1, opp_tri2]
                    // Safety: flap[0] and flap[1] are >= 0 (boundary edges with -1
                    // are skipped above).
                    #[allow(clippy::cast_sign_loss)]
                    let v = [
                        model.flexedge_vert[e][0],
                        model.flexedge_vert[e][1],
                        flap[0] as usize,
                        flap[1] as usize,
                    ];

                    // Runtime edge vectors (from current deformed positions)
                    let xpos = &data.flexvert_xpos;
                    let ed0 = xpos[v[1]] - xpos[v[0]]; // v1 - v0
                    let ed1 = xpos[v[2]] - xpos[v[0]]; // v2 - v0
                    let ed2 = xpos[v[3]] - xpos[v[0]]; // v3 - v0

                    // Curved reference forces (cross products)
                    let frc1 = ed1.cross(&ed2); // (v2-v0) x (v3-v0)
                    let frc2 = ed2.cross(&ed0); // (v3-v0) x (v1-v0)
                    let frc3 = ed0.cross(&ed1); // (v1-v0) x (v2-v0)
                    let frc0 = -(frc1 + frc2 + frc3);
                    let frc = [frc0, frc1, frc2, frc3];

                    // Read velocities for damper
                    let read_vel = |vi: usize| -> Vector3<f64> {
                        let dof = model.flexvert_dofadr[vi];
                        if dof == usize::MAX {
                            Vector3::zeros()
                        } else {
                            Vector3::new(data.qvel[dof], data.qvel[dof + 1], data.qvel[dof + 2])
                        }
                    };
                    let vel = [
                        read_vel(v[0]),
                        read_vel(v[1]),
                        read_vel(v[2]),
                        read_vel(v[3]),
                    ];

                    // B matrix coefficients for this edge
                    let b_base = 17 * e;

                    // Force accumulation: spring[3*i+x] and damper[3*i+x]
                    let mut spring = [0.0f64; 12]; // 4 vertices x 3 axes
                    let mut damper = [0.0f64; 12];

                    for i in 0..4 {
                        for x in 0..3 {
                            for j in 0..4 {
                                let b_ij = model.flex_bending[b_base + 4 * i + j];
                                spring[3 * i + x] += b_ij * xpos[v[j]][x];
                                damper[3 * i + x] += b_ij * vel[j][x];
                            }
                            // Curved reference contribution
                            spring[3 * i + x] += model.flex_bending[b_base + 16] * frc[i][x];
                        }
                    }

                    // Insert forces into qfrc_spring and qfrc_damper
                    for i in 0..4 {
                        let dof = model.flexvert_dofadr[v[i]];
                        if dof == usize::MAX {
                            continue; // pinned vertex: no DOFs
                        }
                        for x in 0..3 {
                            if has_spring {
                                // Spring subtracted (negative sign matches MuJoCo)
                                data.qfrc_spring[dof + x] -= spring[3 * i + x];
                            }
                            if has_damper {
                                // Damper subtracted and multiplied by flex_damping
                                data.qfrc_damper[dof + x] -= damper[3 * i + x] * damping_coeff;
                            }
                        }
                    }
                }
            }
            FlexBendingType::Bridson => {
                // §42B S8: Delegate to Bridson dihedral angle springs (extension).
                apply_bridson_bending(model, data, f, has_spring, has_damper);
            }
        }
    }

    // Fluid forces (§40): compute into qfrc_fluid.
    let has_fluid = mj_fluid(model, data);

    // Gravity compensation (§35): compute into qfrc_gravcomp.
    // Route to qfrc_passive only for DOFs whose joint has jnt_actgravcomp == false.
    // DOFs with jnt_actgravcomp == true are routed via mj_fwd_actuation() (S4.2).
    let has_gravcomp = mj_gravcomp(model, data);

    // S4.7e: Aggregation into qfrc_passive (matches MuJoCo mj_passive() pattern).
    // qfrc_passive = qfrc_spring + qfrc_damper [+ qfrc_gravcomp] [+ qfrc_fluid]
    // Uses assignment (=), not accumulation (+=), for the base sum.
    let agg_count = if sleep_filter {
        data.nv_awake
    } else {
        model.nv
    };
    for idx in 0..agg_count {
        let dof = if sleep_filter {
            data.dof_awake_ind[idx]
        } else {
            idx
        };
        data.qfrc_passive[dof] = data.qfrc_spring[dof] + data.qfrc_damper[dof];
    }
    if has_gravcomp {
        for idx in 0..agg_count {
            let dof = if sleep_filter {
                data.dof_awake_ind[idx]
            } else {
                idx
            };
            // S4.2a: Only route gravcomp to passive for DOFs without actuator routing.
            let jnt = model.dof_jnt[dof];
            if !model.jnt_actgravcomp[jnt] {
                data.qfrc_passive[dof] += data.qfrc_gravcomp[dof];
            }
        }
    }
    if has_fluid {
        for idx in 0..agg_count {
            let dof = if sleep_filter {
                data.dof_awake_ind[idx]
            } else {
                idx
            };
            data.qfrc_passive[dof] += data.qfrc_fluid[dof];
        }
    }

    // DT-101: mj_contactPassive() insertion point.
    // When implemented, call here with guard: disabled(CONTACT) || ncon == 0 || nv == 0.
    // Must go AFTER aggregation because aggregation uses assignment (=), not
    // accumulation (+=) — contact passive forces would be overwritten if placed
    // before. The contact passive contribution should use += on qfrc_passive.

    // DT-21: xfrc_applied projection moved to acceleration stage (qfrc_smooth
    // in compute_qacc_smooth). MuJoCo projects xfrc_applied into qfrc_smooth in
    // mj_fwdAcceleration, not mj_passive.

    // DT-79: Invoke user passive callback (if set).
    if let Some(ref cb) = model.cb_passive {
        (cb.0)(model, data);
    }

    // §66: Plugin passive force dispatch
    if model.nplugin > 0 {
        for i in 0..model.nplugin {
            if model.plugin_capabilities[i].contains(crate::plugin::PluginCapabilityBit::Passive) {
                model.plugin_objects[i].compute(
                    model,
                    data,
                    i,
                    crate::plugin::PluginCapabilityBit::Passive,
                );
            }
        }
    }
}

/// §42B S8: Bridson dihedral angle bending (CortenForge extension).
///
/// Exact algorithm from the pre-Spec-B bending loop, scoped to one flex body.
/// Iterates `flexhinge_*` arrays for the given flex, applying dihedral angle
/// spring/damper forces with per-vertex stability clamp.
fn apply_bridson_bending(
    model: &Model,
    data: &mut Data,
    flex_id: usize,
    has_spring: bool,
    has_damper: bool,
) {
    let dt = model.timestep;
    for h in 0..model.nflexhinge {
        if model.flexhinge_flexid[h] != flex_id {
            continue;
        }
        let [ve0, ve1, va, vb] = model.flexhinge_vert[h];

        let k_bend_raw = model.flex_bend_stiffness[flex_id];
        let b_bend = model.flex_bend_damping[flex_id];
        if k_bend_raw <= 0.0 && b_bend <= 0.0 {
            continue;
        }

        let pe0 = data.flexvert_xpos[ve0];
        let pe1 = data.flexvert_xpos[ve1];
        let pa = data.flexvert_xpos[va];
        let pb = data.flexvert_xpos[vb];
        let rest_angle = model.flexhinge_angle0[h];

        // Shared edge vector
        let e = pe1 - pe0;
        let e_len_sq = e.norm_squared();
        if e_len_sq < 1e-20 {
            continue;
        }

        // Face normals (unnormalized)
        let offset_a = pa - pe0;
        let offset_b = pb - pe0;
        let normal_face_a = e.cross(&offset_a);
        let normal_face_b = offset_b.cross(&e);
        let norm_sq_a = normal_face_a.norm_squared();
        let norm_sq_b = normal_face_b.norm_squared();
        if norm_sq_a < 1e-20 || norm_sq_b < 1e-20 {
            continue;
        }

        // Dihedral angle via atan2
        let e_len = e_len_sq.sqrt();
        let e_norm = e / e_len;
        let normal_unit_a = normal_face_a / norm_sq_a.sqrt();
        let normal_unit_b = normal_face_b / norm_sq_b.sqrt();
        let cos_theta = normal_unit_a.dot(&normal_unit_b).clamp(-1.0, 1.0);
        let sin_theta = normal_unit_a.cross(&normal_unit_b).dot(&e_norm);
        let theta = sin_theta.atan2(cos_theta);
        let angle_error = theta - rest_angle;

        // Bridson dihedral gradient (all 4 vertices)
        let grad_a = e_len * normal_face_a / norm_sq_a;
        let grad_b = e_len * normal_face_b / norm_sq_b;
        let bary_a = offset_a.dot(&e) / e_len_sq;
        let bary_b = offset_b.dot(&e) / e_len_sq;
        let grad_e0 = -grad_a * (1.0 - bary_a) - grad_b * (1.0 - bary_b);
        let grad_e1 = -grad_a * bary_a - grad_b * bary_b;

        // S4.7b: Spring gated on DISABLE_SPRING.
        let spring_mag = if has_spring {
            -k_bend_raw * angle_error
        } else {
            0.0
        };

        // Damper: F = -b * d(theta)/dt, where d(theta)/dt = J · qvel
        // (§27F) Pinned vertices have dofadr=usize::MAX and zero velocity.
        let dof_e0 = model.flexvert_dofadr[ve0];
        let dof_e1 = model.flexvert_dofadr[ve1];
        let dof_a = model.flexvert_dofadr[va];
        let dof_b = model.flexvert_dofadr[vb];
        let read_vel = |dof: usize| -> Vector3<f64> {
            if dof == usize::MAX {
                Vector3::zeros()
            } else {
                Vector3::new(data.qvel[dof], data.qvel[dof + 1], data.qvel[dof + 2])
            }
        };
        let vel_e0 = read_vel(dof_e0);
        let vel_e1 = read_vel(dof_e1);
        let vel_a = read_vel(dof_a);
        let vel_b = read_vel(dof_b);
        let theta_dot =
            grad_e0.dot(&vel_e0) + grad_e1.dot(&vel_e1) + grad_a.dot(&vel_a) + grad_b.dot(&vel_b);
        // S4.7b: Damper gated on DISABLE_DAMPER.
        let damper_mag = if has_damper { -b_bend * theta_dot } else { 0.0 };

        // S4.7-prereq: Apply spring and damper to separate arrays.
        // Per-vertex stability clamp applied independently to each component.
        let grads = [
            (ve0, dof_e0, grad_e0),
            (ve1, dof_e1, grad_e1),
            (va, dof_a, grad_a),
            (vb, dof_b, grad_b),
        ];
        for &(v_idx, dof, grad) in &grads {
            let invmass = model.flexvert_invmass[v_idx];
            if invmass > 0.0 {
                let grad_norm = grad.norm();
                if spring_mag != 0.0 {
                    let mut fm = spring_mag;
                    if grad_norm > 0.0 {
                        let fm_max = 1.0 / (dt * dt * grad_norm * invmass);
                        fm = fm.clamp(-fm_max, fm_max);
                    }
                    for ax in 0..3 {
                        data.qfrc_spring[dof + ax] += grad[ax] * fm;
                    }
                }
                if damper_mag != 0.0 {
                    let mut fm = damper_mag;
                    if grad_norm > 0.0 {
                        let fm_max = 1.0 / (dt * dt * grad_norm * invmass);
                        fm = fm.clamp(-fm_max, fm_max);
                    }
                    for ax in 0..3 {
                        data.qfrc_damper[dof + ax] += grad[ax] * fm;
                    }
                }
            }
        }
    }
}

/// Visitor for computing passive forces (springs, dampers).
#[allow(clippy::struct_excessive_bools)] // 4 independent configuration flags
struct PassiveForceVisitor<'a> {
    model: &'a Model,
    data: &'a mut Data,
    implicit_mode: bool,
    sleep_enabled: bool,
    /// S4.7b: false when DISABLE_SPRING is set.
    has_spring: bool,
    /// S4.7b: false when DISABLE_DAMPER is set.
    has_damper: bool,
}

impl PassiveForceVisitor<'_> {
    /// Check if a joint's body is sleeping (§16.5a').
    #[inline]
    fn is_joint_sleeping(&self, ctx: &JointContext) -> bool {
        self.sleep_enabled
            && self.data.body_sleep_state[self.model.jnt_body[ctx.jnt_id]] == SleepState::Asleep
    }

    /// Process a 1-DOF joint (Hinge or Slide) with spring and damper.
    /// S4.7-prereq: writes to `qfrc_spring`/`qfrc_damper` instead of `qfrc_passive`.
    #[inline]
    fn visit_1dof_joint(&mut self, ctx: JointContext) {
        if self.is_joint_sleeping(&ctx) {
            return;
        }
        let dof_adr = ctx.dof_adr;
        let qpos_adr = ctx.qpos_adr;
        let jnt_id = ctx.jnt_id;

        if !self.implicit_mode {
            // S4.7b: Spring gated on DISABLE_SPRING.
            if self.has_spring {
                let stiffness = self.model.jnt_stiffness[jnt_id];
                let springref = self.model.qpos_spring[qpos_adr];
                let q = self.data.qpos[qpos_adr];
                self.data.qfrc_spring[dof_adr] -= stiffness * (q - springref);
            }

            // S4.7b: Damper gated on DISABLE_DAMPER.
            if self.has_damper {
                let damping = self.model.jnt_damping[jnt_id];
                let qvel = self.data.qvel[dof_adr];
                self.data.qfrc_damper[dof_adr] -= damping * qvel;
            }
        }
    }

    /// Process a multi-DOF joint (Ball or Free) with spring + damper.
    /// Spring force: `-k * subquat(q, q_spring)` for rotational DOFs,
    /// `-k * (pos - pos_spring)` for translational DOFs (free joint only).
    /// S4.7-prereq: writes to `qfrc_spring`/`qfrc_damper` instead of `qfrc_passive`.
    #[inline]
    fn visit_multi_dof_joint(&mut self, ctx: JointContext) {
        if self.is_joint_sleeping(&ctx) {
            return;
        }

        // Spring force: gated on DISABLE_SPRING and implicit_mode.
        // Ball/free spring forces are fully suppressed in ImplicitSpringDamper
        // mode — matching MuJoCo (quaternion springs too nonlinear for diagonal
        // implicit approximation; implicit_stiffness=0.0 for ball/free).
        if !self.implicit_mode && self.has_spring {
            let stiffness = self.model.jnt_stiffness[ctx.jnt_id];
            if stiffness != 0.0 {
                match ctx.jnt_type {
                    MjJointType::Free => {
                        // Translational: -k * (pos - pos_spring) for 3 DOFs
                        for i in 0..3 {
                            let q = self.data.qpos[ctx.qpos_adr + i];
                            let q_spring = self.model.qpos_spring[ctx.qpos_adr + i];
                            self.data.qfrc_spring[ctx.dof_adr + i] -= stiffness * (q - q_spring);
                        }
                        // Rotational: falls through to ball case.
                        // Quat starts at qpos_adr + 3, rotational DOFs at dof_adr + 3.
                        let quat_adr = ctx.qpos_adr + 3;
                        let rot_dof_adr = ctx.dof_adr + 3;
                        let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                            self.data.qpos[quat_adr],
                            self.data.qpos[quat_adr + 1],
                            self.data.qpos[quat_adr + 2],
                            self.data.qpos[quat_adr + 3],
                        ));
                        let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                            self.model.qpos_spring[quat_adr],
                            self.model.qpos_spring[quat_adr + 1],
                            self.model.qpos_spring[quat_adr + 2],
                            self.model.qpos_spring[quat_adr + 3],
                        ));
                        let dif = subquat(&q_cur, &q_spring);
                        self.data.qfrc_spring[rot_dof_adr] -= stiffness * dif[0];
                        self.data.qfrc_spring[rot_dof_adr + 1] -= stiffness * dif[1];
                        self.data.qfrc_spring[rot_dof_adr + 2] -= stiffness * dif[2];
                    }
                    MjJointType::Ball => {
                        let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                            self.data.qpos[ctx.qpos_adr],
                            self.data.qpos[ctx.qpos_adr + 1],
                            self.data.qpos[ctx.qpos_adr + 2],
                            self.data.qpos[ctx.qpos_adr + 3],
                        ));
                        let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                            self.model.qpos_spring[ctx.qpos_adr],
                            self.model.qpos_spring[ctx.qpos_adr + 1],
                            self.model.qpos_spring[ctx.qpos_adr + 2],
                            self.model.qpos_spring[ctx.qpos_adr + 3],
                        ));
                        let dif = subquat(&q_cur, &q_spring);
                        self.data.qfrc_spring[ctx.dof_adr] -= stiffness * dif[0];
                        self.data.qfrc_spring[ctx.dof_adr + 1] -= stiffness * dif[1];
                        self.data.qfrc_spring[ctx.dof_adr + 2] -= stiffness * dif[2];
                    }
                    _ => {} // Hinge/Slide handled by visit_1dof_joint
                }
            }
        }

        // Damper (existing code, unchanged).
        for i in 0..ctx.nv {
            let dof_idx = ctx.dof_adr + i;
            // S4.7b: Damper gated on DISABLE_DAMPER.
            if !self.implicit_mode && self.has_damper {
                let dof_damping = self.model.dof_damping[dof_idx];
                let qvel = self.data.qvel[dof_idx];
                self.data.qfrc_damper[dof_idx] -= dof_damping * qvel;
            }
        }
    }
}

impl JointVisitor for PassiveForceVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        self.visit_1dof_joint(ctx);
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        self.visit_1dof_joint(ctx);
    }

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        self.visit_multi_dof_joint(ctx);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        self.visit_multi_dof_joint(ctx);
    }
}
