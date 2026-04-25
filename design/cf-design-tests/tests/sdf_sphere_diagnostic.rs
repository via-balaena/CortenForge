//! SDF sphere stacking diagnostic — compares cf-design's `to_model()` output
//! against an MJCF-loaded model with the same SDF sphere geom, field-by-field.
//!
//! Held in `cf-design-tests` (L0-integration tier) so the sim-mjcf dev-dep
//! stays out of cf-design's L0 dev-graph. Originally inline in
//! cf-design/src/mechanism/model_builder.rs.

// Mirrors the file-level allows on cf-design/src/mechanism/model_builder.rs
// where this diagnostic test originally lived; eprintln!-heavy debug code
// trips many pedantic lints that were tolerated for the same reason there.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::similar_names,
    clippy::unreadable_literal,
    clippy::too_many_lines,
    clippy::cognitive_complexity,
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::clone_on_copy,
    clippy::or_fun_call,
    clippy::uninlined_format_args
)]

use cf_design::{JointDef, JointKind, Material, Mechanism, Part, Solid};
use nalgebra::{Point3, Vector3};
use sim_core::{GeomType, ShapeSphere};

/// Diagnostic: compare cf-design model vs MJCF model for the same sphere pair.
///
/// Findings so far:
/// - `diagapprox_bodyweight` was false (fixed in generate) — improved settling
/// - Mass/inertia differ by 1-3.6% (SDF integration vs analytical) — not sole cause
/// - SDF grid, geom ordering, mesh geoms — all ruled out individually
/// - A remaining Model field diff causes late instability (~step 1400)
/// - Next step: serialization-based exhaustive Model field diff
#[test]
// Domain notation preserves geometric conventions.
#[allow(
    clippy::similar_names,
    clippy::unreadable_literal,
    unused_variables,
    clippy::too_many_lines,
    clippy::cognitive_complexity,
    clippy::redundant_clone,
    clippy::cast_lossless,
    clippy::clone_on_copy,
    clippy::or_fun_call
)]
fn diagnose_sdf_sphere_gap() {
    use sim_core::SdfGrid;
    use std::sync::Arc;

    // ── Step 1: Compare SDF grids ──────────────────────────────────
    let conformance_sdf = SdfGrid::sphere(Point3::origin(), 5.0, 20, 2.0);
    let cfdesign_sdf = Solid::sphere(5.0).sdf_grid_at(1.0).unwrap();

    eprintln!("=== Step 1: SDF Grid Comparison ===");
    eprintln!(
        "  conformance: cell_size={:.4} dims={}x{}x{} origin=({:.2},{:.2},{:.2})",
        conformance_sdf.cell_size(),
        conformance_sdf.width(),
        conformance_sdf.height(),
        conformance_sdf.depth(),
        conformance_sdf.origin().x,
        conformance_sdf.origin().y,
        conformance_sdf.origin().z,
    );
    eprintln!(
        "  cf-design:   cell_size={:.4} dims={}x{}x{} origin=({:.2},{:.2},{:.2})",
        cfdesign_sdf.cell_size(),
        cfdesign_sdf.width(),
        cfdesign_sdf.height(),
        cfdesign_sdf.depth(),
        cfdesign_sdf.origin().x,
        cfdesign_sdf.origin().y,
        cfdesign_sdf.origin().z,
    );

    // Ray-march radius along Z (the stacking axis)
    let ray_radius = |sdf: &SdfGrid| -> f64 {
        let dir = nalgebra::Vector3::z();
        let mut t = 0.0_f64;
        for _ in 0..200 {
            let point = Point3::from(dir * t);
            let Some(dist) = sdf.distance(point) else {
                return t;
            };
            if dist >= 0.0 {
                return t;
            }
            t += (-dist).max(sdf.cell_size() * 0.01);
        }
        t
    };
    let r_conf = ray_radius(&conformance_sdf);
    let r_cfd = ray_radius(&cfdesign_sdf);
    eprintln!(
        "  ray_radius_z: conformance={:.4} cf-design={:.4} delta={:.4}",
        r_conf,
        r_cfd,
        (r_conf - r_cfd).abs()
    );

    // ── Step 2: Build both models and diff fields ──────────────────
    eprintln!("\n=== Step 2: Model Field Comparison ===");

    // MJCF model (conformance path)
    let mjcf = r#"
        <mujoco model="diag_mjcf">
            <option gravity="0 0 -9810" timestep="0.0005"/>
            <worldbody>
                <geom name="floor" type="plane" size="40 40 0.1"/>
                <body name="lower" pos="0 0 5.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom name="lo_sphere" type="sphere" size="5"/>
                </body>
                <body name="upper" pos="0 0 15.5">
                    <joint type="free"/>
                    <inertial pos="0 0 0" mass="0.000655" diaginertia="0.00655 0.00655 0.00655"/>
                    <geom name="up_sphere" type="sphere" size="5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let mut mjcf_model = sim_mjcf::load_model(mjcf).expect("load MJCF");
    // Swap sphere geoms to SDF (same as conformance test)
    let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 5.0, 20, 2.0));
    mjcf_model
        .shape_data
        .push(Arc::new(ShapeSphere::new(grid.clone(), 5.0)));
    mjcf_model
        .shape_data
        .push(Arc::new(ShapeSphere::new(grid, 5.0)));
    mjcf_model.nshape = 2;
    for geom_id in 1..=2 {
        mjcf_model.geom_type[geom_id] = GeomType::Sdf;
        mjcf_model.geom_shape[geom_id] = Some(geom_id - 1);
    }

    // cf-design model
    let material = Material::new("PLA", 1250.0);
    let m = Mechanism::builder("pair")
        .part(Part::new("lower", Solid::sphere(5.0), material.clone()))
        .part(Part::new("upper", Solid::sphere(5.0), material))
        .joint(JointDef::new(
            "free_lower",
            "world",
            "lower",
            JointKind::Free,
            Point3::new(0.0, 0.0, 5.5),
            Vector3::z(),
        ))
        .joint(JointDef::new(
            "free_upper",
            "world",
            "upper",
            JointKind::Free,
            Point3::new(0.0, 0.0, 15.5),
            Vector3::z(),
        ))
        .build();
    let mut cfd_model = m.to_model(1.0, 0.3);
    cfd_model.add_ground_plane();

    // Compare key fields for bodies 1 and 2 (the spheres)
    eprintln!("  {:>25} {:>15} {:>15}", "field", "mjcf", "cf-design");
    eprintln!("  {:-<25} {:-<15} {:-<15}", "", "", "");

    eprintln!(
        "  nbody:                  {:>15} {:>15}",
        mjcf_model.nbody, cfd_model.nbody
    );
    eprintln!(
        "  ngeom:                  {:>15} {:>15}",
        mjcf_model.ngeom, cfd_model.ngeom
    );
    eprintln!(
        "  njnt:                   {:>15} {:>15}",
        mjcf_model.njnt, cfd_model.njnt
    );
    eprintln!(
        "  nq:                     {:>15} {:>15}",
        mjcf_model.nq, cfd_model.nq
    );
    eprintln!(
        "  nv:                     {:>15} {:>15}",
        mjcf_model.nv, cfd_model.nv
    );
    eprintln!(
        "  timestep:               {:>15.6} {:>15.6}",
        mjcf_model.timestep, cfd_model.timestep
    );
    eprintln!(
        "  gravity_z:              {:>15.1} {:>15.1}",
        mjcf_model.gravity.z, cfd_model.gravity.z
    );

    for body_id in 1..3 {
        let bname_m = mjcf_model
            .body_name
            .get(body_id)
            .and_then(|n| n.as_deref())
            .unwrap_or("?");
        let bname_c = cfd_model
            .body_name
            .get(body_id)
            .and_then(|n| n.as_deref())
            .unwrap_or("?");
        eprintln!(
            "\n  --- body {} (mjcf='{}' cfd='{}') ---",
            body_id, bname_m, bname_c
        );
        eprintln!(
            "  body_mass:              {:>15.6} {:>15.6}",
            mjcf_model.body_mass[body_id], cfd_model.body_mass[body_id]
        );
        eprintln!(
            "  body_inertia:           {:>15.6} {:>15.6}",
            mjcf_model.body_inertia[body_id].x, cfd_model.body_inertia[body_id].x
        );
        eprintln!(
            "  body_pos:               ({:.3},{:.3},{:.3})  ({:.3},{:.3},{:.3})",
            mjcf_model.body_pos[body_id].x,
            mjcf_model.body_pos[body_id].y,
            mjcf_model.body_pos[body_id].z,
            cfd_model.body_pos[body_id].x,
            cfd_model.body_pos[body_id].y,
            cfd_model.body_pos[body_id].z
        );
        eprintln!(
            "  body_ipos:              ({:.4},{:.4},{:.4})  ({:.4},{:.4},{:.4})",
            mjcf_model.body_ipos[body_id].x,
            mjcf_model.body_ipos[body_id].y,
            mjcf_model.body_ipos[body_id].z,
            cfd_model.body_ipos[body_id].x,
            cfd_model.body_ipos[body_id].y,
            cfd_model.body_ipos[body_id].z
        );
        eprintln!(
            "  body_invweight0:        [{:.6},{:.6}]  [{:.6},{:.6}]",
            mjcf_model.body_invweight0[body_id][0],
            mjcf_model.body_invweight0[body_id][1],
            cfd_model.body_invweight0[body_id][0],
            cfd_model.body_invweight0[body_id][1]
        );
        eprintln!(
            "  body_subtreemass:       {:>15.6} {:>15.6}",
            mjcf_model.body_subtreemass[body_id], cfd_model.body_subtreemass[body_id]
        );
        eprintln!(
            "  body_parent:            {:>15} {:>15}",
            mjcf_model.body_parent[body_id], cfd_model.body_parent[body_id]
        );
    }

    // Compare geom fields (SDF geoms only)
    eprintln!("\n  === Geom comparison (SDF geoms) ===");
    let mjcf_sdf_geoms: Vec<usize> = (0..mjcf_model.ngeom)
        .filter(|&g| mjcf_model.geom_type[g] == GeomType::Sdf)
        .collect();
    let cfd_sdf_geoms: Vec<usize> = (0..cfd_model.ngeom)
        .filter(|&g| cfd_model.geom_type[g] == GeomType::Sdf)
        .collect();
    eprintln!(
        "  SDF geom count: mjcf={} cfd={}",
        mjcf_sdf_geoms.len(),
        cfd_sdf_geoms.len()
    );

    for (i, (&mg, &cg)) in mjcf_sdf_geoms.iter().zip(cfd_sdf_geoms.iter()).enumerate() {
        eprintln!("\n  SDF geom pair {} (mjcf={} cfd={}):", i, mg, cg);
        eprintln!(
            "    geom_body:     {} vs {}",
            mjcf_model.geom_body[mg], cfd_model.geom_body[cg]
        );
        eprintln!(
            "    geom_pos:      ({:.3},{:.3},{:.3}) vs ({:.3},{:.3},{:.3})",
            mjcf_model.geom_pos[mg].x,
            mjcf_model.geom_pos[mg].y,
            mjcf_model.geom_pos[mg].z,
            cfd_model.geom_pos[cg].x,
            cfd_model.geom_pos[cg].y,
            cfd_model.geom_pos[cg].z
        );
        eprintln!(
            "    geom_size:     ({:.3},{:.3},{:.3}) vs ({:.3},{:.3},{:.3})",
            mjcf_model.geom_size[mg].x,
            mjcf_model.geom_size[mg].y,
            mjcf_model.geom_size[mg].z,
            cfd_model.geom_size[cg].x,
            cfd_model.geom_size[cg].y,
            cfd_model.geom_size[cg].z
        );
        eprintln!(
            "    geom_margin:   {:.4} vs {:.4}",
            mjcf_model.geom_margin[mg], cfd_model.geom_margin[cg]
        );
        eprintln!(
            "    geom_solref:   {:?} vs {:?}",
            mjcf_model.geom_solref[mg], cfd_model.geom_solref[cg]
        );
        eprintln!(
            "    geom_solimp:   {:?} vs {:?}",
            mjcf_model.geom_solimp[mg], cfd_model.geom_solimp[cg]
        );
        eprintln!(
            "    geom_friction: ({:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4})",
            mjcf_model.geom_friction[mg].x,
            mjcf_model.geom_friction[mg].y,
            mjcf_model.geom_friction[mg].z,
            cfd_model.geom_friction[cg].x,
            cfd_model.geom_friction[cg].y,
            cfd_model.geom_friction[cg].z
        );
        eprintln!(
            "    geom_condim:   {} vs {}",
            mjcf_model.geom_condim[mg], cfd_model.geom_condim[cg]
        );
        eprintln!(
            "    geom_contype:  {} vs {}",
            mjcf_model.geom_contype[mg], cfd_model.geom_contype[cg]
        );
        eprintln!(
            "    geom_rbound:   {:.4} vs {:.4}",
            mjcf_model.geom_rbound[mg], cfd_model.geom_rbound[cg]
        );
    }

    // Compare initial qpos
    eprintln!("\n  === Initial qpos ===");
    let nq = mjcf_model.nq.min(cfd_model.nq);
    for i in 0..nq {
        let mq = mjcf_model.qpos0[i];
        let cq = cfd_model.qpos0[i];
        if (mq - cq).abs() > 1e-10 {
            eprintln!(
                "  qpos0[{}]: mjcf={:.6} cfd={:.6} DIFF={:.6}",
                i,
                mq,
                cq,
                mq - cq
            );
        }
    }

    // Step 2b: Clone the MJCF model exactly — rebuild it from scratch
    // using the same approach as the conformance test. This isolates
    // whether the issue is in Model construction or something else.
    let mut cfd_patched = mjcf_model.clone();

    // Only difference: use cf-design's SDF grids instead of conformance
    let cfd_sdf_grid = Arc::new(Solid::sphere(5.0).sdf_grid_at(1.0).unwrap());
    cfd_patched.shape_data = vec![
        Arc::new(ShapeSphere::new(cfd_sdf_grid.clone(), 5.0)),
        Arc::new(ShapeSphere::new(cfd_sdf_grid, 5.0)),
    ];

    // Test: add dummy mesh geoms to MJCF model (same structure as cf-design)
    // If this breaks stability, the mesh geoms interfere despite contype=0.
    let mut mjcf_with_mesh = mjcf_model.clone();
    // Insert mesh geom after each SDF geom (to match cf-design layout)
    // For simplicity, just add 2 mesh geoms at the end with contype=0
    for _ in 0..2 {
        mjcf_with_mesh.geom_type.push(GeomType::Mesh);
        mjcf_with_mesh.geom_body.push(1);
        mjcf_with_mesh.geom_pos.push(Vector3::zeros());
        mjcf_with_mesh
            .geom_quat
            .push(nalgebra::UnitQuaternion::identity());
        mjcf_with_mesh.geom_size.push(Vector3::new(1.0, 1.0, 1.0));
        mjcf_with_mesh
            .geom_friction
            .push(Vector3::new(1.0, 0.005, 0.0001));
        mjcf_with_mesh.geom_condim.push(3);
        mjcf_with_mesh.geom_contype.push(0);
        mjcf_with_mesh.geom_conaffinity.push(0);
        mjcf_with_mesh.geom_margin.push(0.0);
        mjcf_with_mesh.geom_gap.push(0.0);
        mjcf_with_mesh.geom_priority.push(0);
        mjcf_with_mesh.geom_solmix.push(1.0);
        mjcf_with_mesh
            .geom_solimp
            .push([0.9, 0.95, 0.001, 0.5, 2.0]);
        mjcf_with_mesh.geom_solref.push([0.02, 1.0]);
        mjcf_with_mesh.geom_name.push(None);
        mjcf_with_mesh.geom_rbound.push(10.0);
        mjcf_with_mesh.geom_aabb.push([0.0; 6]);
        mjcf_with_mesh.geom_mesh.push(None);
        mjcf_with_mesh.geom_hfield.push(None);
        mjcf_with_mesh.geom_shape.push(None);
        mjcf_with_mesh.geom_group.push(0);
        mjcf_with_mesh.geom_rgba.push([0.9, 0.9, 0.9, 1.0]);
        mjcf_with_mesh.geom_fluid.push([0.0; 12]);
        mjcf_with_mesh.geom_plugin.push(None);
        mjcf_with_mesh.geom_user.push(vec![]);
    }
    mjcf_with_mesh.ngeom += 2;
    mjcf_with_mesh.body_geom_num[1] += 1;
    mjcf_with_mesh.body_geom_num[2] += 1;

    let mut mesh_data = mjcf_with_mesh.make_data();
    mesh_data.forward(&mjcf_with_mesh).unwrap();
    for step in 0..2000 {
        mesh_data.step(&mjcf_with_mesh).unwrap();
    }
    eprintln!(
        "  MJCF+mesh: z_lo={:.3} z_up={:.3} gap={:.3} ncon={}",
        mesh_data.qpos[2],
        mesh_data.qpos[9],
        mesh_data.qpos[9] - mesh_data.qpos[2],
        mesh_data.ncon
    );

    eprintln!("\n  === Patched: MJCF clone + cf-design SDF grid (STABLE) ===");
    eprintln!("  ngeom:        {}", cfd_patched.ngeom);

    // Now build the REAL cf-design model with same SDF + mass as MJCF
    // and diff EVERY field to find what cf-design sets wrong.
    let mut cfd_full = m.to_model(1.0, 0.3);
    cfd_full.add_ground_plane();
    // Patch to match MJCF exactly (mass, inertia, SDF)
    let cfd_sdf2 = Arc::new(Solid::sphere(5.0).sdf_grid_at(1.0).unwrap());
    cfd_full.shape_data = vec![
        Arc::new(ShapeSphere::new(cfd_sdf2.clone(), 5.0)),
        Arc::new(ShapeSphere::new(cfd_sdf2, 5.0)),
    ];
    for body_id in 1..cfd_full.nbody {
        cfd_full.body_mass[body_id] = 0.000655;
        cfd_full.body_inertia[body_id] = Vector3::new(0.00655, 0.00655, 0.00655);
    }
    cfd_full.compute_implicit_params();
    cfd_full.compute_invweight0();

    // Both have same mass, inertia, SDF, invweight0.
    // Diff everything else.
    eprintln!("\n  === Field-by-field diff: MJCF-clone(stable) vs cfd_full(unstable) ===");
    eprintln!("  ngeom: {} vs {}", cfd_patched.ngeom, cfd_full.ngeom);
    eprintln!("  nbody: {} vs {}", cfd_patched.nbody, cfd_full.nbody);

    // Geom-level diffs (all geoms)
    let max_geom = cfd_patched.ngeom.max(cfd_full.ngeom);
    for g in 0..max_geom {
        let in_p = g < cfd_patched.ngeom;
        let in_c = g < cfd_full.ngeom;
        if in_p && in_c {
            let tp = cfd_patched.geom_type[g];
            let tc = cfd_full.geom_type[g];
            if tp != tc {
                eprintln!("  geom[{}] type: {:?} vs {:?}", g, tp, tc);
            }
            let bp = cfd_patched.geom_body[g];
            let bc = cfd_full.geom_body[g];
            if bp != bc {
                eprintln!("  geom[{}] body: {} vs {}", g, bp, bc);
            }
            let ct_p = cfd_patched.geom_contype[g];
            let ct_c = cfd_full.geom_contype[g];
            let ca_p = cfd_patched.geom_conaffinity[g];
            let ca_c = cfd_full.geom_conaffinity[g];
            if ct_p != ct_c || ca_p != ca_c {
                eprintln!(
                    "  geom[{}] contype/aff: {}/{} vs {}/{}",
                    g, ct_p, ca_p, ct_c, ca_c
                );
            }
        } else {
            let which = if in_p { "patched-only" } else { "cfd-only" };
            let model = if in_p { &cfd_patched } else { &cfd_full };
            eprintln!(
                "  geom[{}] {}: type={:?} body={} contype={} conaff={}",
                g,
                which,
                model.geom_type[g],
                model.geom_body[g],
                model.geom_contype[g],
                model.geom_conaffinity[g]
            );
        }
    }

    // Body geom address/count
    for b in 0..cfd_patched.nbody {
        let a1 = cfd_patched.body_geom_adr[b];
        let n1 = cfd_patched.body_geom_num[b];
        let a2 = cfd_full.body_geom_adr[b];
        let n2 = cfd_full.body_geom_num[b];
        if a1 != a2 || n1 != n2 {
            eprintln!("  body[{}] geom_adr/num: {}/{} vs {}/{}", b, a1, n1, a2, n2);
        }
    }

    // Option/solver fields
    let fields: Vec<(&str, f64, f64)> = vec![
        ("cone", cfd_patched.cone as f64, cfd_full.cone as f64),
        ("impratio", cfd_patched.impratio, cfd_full.impratio),
        (
            "solver_iter",
            cfd_patched.solver_iterations as f64,
            cfd_full.solver_iterations as f64,
        ),
        (
            "sdf_iter",
            cfd_patched.sdf_iterations as f64,
            cfd_full.sdf_iterations as f64,
        ),
        (
            "sdf_initpoints",
            cfd_patched.sdf_initpoints as f64,
            cfd_full.sdf_initpoints as f64,
        ),
        (
            "disableflags",
            cfd_patched.disableflags as f64,
            cfd_full.disableflags as f64,
        ),
        (
            "enableflags",
            cfd_patched.enableflags as f64,
            cfd_full.enableflags as f64,
        ),
    ];
    for (name, v1, v2) in &fields {
        if (v1 - v2).abs() > 1e-12 {
            eprintln!("  {}: {} vs {}", name, v1, v2);
        }
    }

    // Step 3: run all three models and compare
    eprintln!("\n=== Step 3: 2000-step comparison (mjcf vs cfd vs cfd_patched) ===");
    // DEFINITIVE TEST: Take MJCF model, rearrange to match cf-design layout
    // (SDF geoms first, plane last), keeping everything else from MJCF.
    // If this fails → structure matters. If stable → some field in generate() is wrong.
    let mut mjcf_reordered = mjcf_model.clone();

    // Current layout: [Plane(g0,body0), Sdf(g1,body1), Sdf(g2,body2)]
    // Target layout:  [Sdf(g0,body1), Sdf(g1,body2), Plane(g2,body0)]
    // (No mesh geoms yet — just reorder)
    let swap_geom_fields = |m: &mut sim_core::Model, a: usize, b: usize| {
        m.geom_type.swap(a, b);
        m.geom_body.swap(a, b);
        m.geom_pos.swap(a, b);
        m.geom_quat.swap(a, b);
        m.geom_size.swap(a, b);
        m.geom_friction.swap(a, b);
        m.geom_condim.swap(a, b);
        m.geom_contype.swap(a, b);
        m.geom_conaffinity.swap(a, b);
        m.geom_margin.swap(a, b);
        m.geom_gap.swap(a, b);
        m.geom_priority.swap(a, b);
        m.geom_solmix.swap(a, b);
        m.geom_solimp.swap(a, b);
        m.geom_solref.swap(a, b);
        m.geom_name.swap(a, b);
        m.geom_rbound.swap(a, b);
        m.geom_aabb.swap(a, b);
        m.geom_mesh.swap(a, b);
        m.geom_hfield.swap(a, b);
        m.geom_shape.swap(a, b);
        m.geom_group.swap(a, b);
        m.geom_rgba.swap(a, b);
        m.geom_fluid.swap(a, b);
        m.geom_plugin.swap(a, b);
        m.geom_user.swap(a, b);
    };

    // Move Plane from g0 to g2: swap g0↔g2, then swap g0↔g1
    swap_geom_fields(&mut mjcf_reordered, 0, 2);
    swap_geom_fields(&mut mjcf_reordered, 0, 1);
    // Now: [Sdf(body1), Sdf(body2), Plane(body0)]
    // Update body_geom_adr/num
    mjcf_reordered.body_geom_adr[0] = 2; // world body has plane at g2
    mjcf_reordered.body_geom_adr[1] = 0; // lower body has sdf at g0
    mjcf_reordered.body_geom_adr[2] = 1; // upper body has sdf at g1

    eprintln!("\n  === MJCF reordered geom layout ===");
    for g in 0..mjcf_reordered.ngeom {
        eprintln!(
            "    g{}: {:?} body={}",
            g, mjcf_reordered.geom_type[g], mjcf_reordered.geom_body[g]
        );
    }

    let mut reord_data = mjcf_reordered.make_data();
    reord_data.forward(&mjcf_reordered).unwrap();
    for _ in 0..2000 {
        reord_data.step(&mjcf_reordered).unwrap();
    }
    let gap_reord = reord_data.qpos[9] - reord_data.qpos[2];
    eprintln!(
        "  MJCF-reordered: z_lo={:.3} z_up={:.3} gap={:.3} ncon={} {}",
        reord_data.qpos[2],
        reord_data.qpos[9],
        gap_reord,
        reord_data.ncon,
        if gap_reord > 8.0 { "STABLE" } else { "FAILED" }
    );

    // REVERSE APPROACH: Start from MJCF (stable), apply cf-design changes
    // one at a time to find which breaks stability.
    let test_model = |name: &str, model: &sim_core::Model| {
        let mut d = model.make_data();
        d.forward(model).unwrap();
        for _ in 0..2000 {
            d.step(model).unwrap();
        }
        let gap = d.qpos[9] - d.qpos[2];
        let status = if gap > 8.0 { "STABLE" } else { "FAILED" };
        eprintln!("  {}: gap={:.3} {}", name, gap, status);
    };

    eprintln!("\n  === Reverse: MJCF + progressive cf-design changes ===");
    // Baseline
    test_model("mjcf_baseline", &cfd_patched); // MJCF clone + cf-design SDF (already stable)

    // 1. Set diagapprox = false (cf-design default before fix)
    let mut m1 = cfd_patched.clone();
    m1.diagapprox_bodyweight = false;
    test_model("+ diagapprox=false", &m1);

    // 2. Also set mass/inertia to cf-design values
    let mut m2 = m1.clone();
    for b in 1..m2.nbody {
        m2.body_mass[b] = cfd_model.body_mass[b];
        m2.body_inertia[b] = cfd_model.body_inertia[b];
    }
    m2.compute_implicit_params();
    m2.compute_stat_meaninertia();
    m2.compute_invweight0();
    test_model("+ cfd mass/inertia", &m2);

    // 3. Start over: MJCF + just diagapprox=false (no mass change)
    let mut m3 = cfd_patched.clone();
    m3.diagapprox_bodyweight = false;
    test_model("diagapprox=false only", &m3);

    // 4. MJCF + just mass change (no diagapprox)
    let mut m4 = cfd_patched.clone();
    for b in 1..m4.nbody {
        m4.body_mass[b] = cfd_model.body_mass[b];
        m4.body_inertia[b] = cfd_model.body_inertia[b];
    }
    m4.compute_implicit_params();
    m4.compute_stat_meaninertia();
    m4.compute_invweight0();
    test_model("cfd mass only", &m4);

    // COMPREHENSIVE DIFF: cfd_patched (MJCF+cfSDF, stable) vs cfd_model (unstable)
    // cfd_patched has diagapprox=true, MJCF mass, cf-design SDF.
    // cfd_model has diagapprox=true (now), cf-design mass, cf-design SDF.
    // Apply cf-design mass to cfd_patched so mass matches.
    let mut m1_model = cfd_patched.clone();
    for b in 1..m1_model.nbody {
        m1_model.body_mass[b] = cfd_model.body_mass[b];
        m1_model.body_inertia[b] = cfd_model.body_inertia[b];
    }
    m1_model.compute_implicit_params();
    m1_model.compute_stat_meaninertia();
    m1_model.compute_invweight0();
    let m1 = &m1_model;
    let m2 = &cfd_model;

    eprintln!("\n  === COMPREHENSIVE DIFF: mjcf_reord vs cfd ===");

    // Joint-level
    for j in 0..m1.njnt.min(m2.njnt) {
        let fields: Vec<(&str, f64, f64)> = vec![
            (
                "jnt_stiffness",
                m1.jnt_stiffness.get(j).copied().unwrap_or(0.0),
                m2.jnt_stiffness.get(j).copied().unwrap_or(0.0),
            ),
            (
                "jnt_armature",
                m1.jnt_armature.get(j).copied().unwrap_or(0.0),
                m2.jnt_armature.get(j).copied().unwrap_or(0.0),
            ),
            (
                "jnt_damping",
                m1.jnt_damping.get(j).copied().unwrap_or(0.0),
                m2.jnt_damping.get(j).copied().unwrap_or(0.0),
            ),
            (
                "jnt_margin",
                m1.jnt_margin.get(j).copied().unwrap_or(0.0),
                m2.jnt_margin.get(j).copied().unwrap_or(0.0),
            ),
        ];
        for (name, v1, v2) in fields {
            if (v1 - v2).abs() > 1e-12 {
                eprintln!("  jnt[{}].{}: {:.8} vs {:.8}", j, name, v1, v2);
            }
        }
    }

    // DOF-level
    for d in 0..m1.nv.min(m2.nv) {
        let fields: Vec<(&str, f64, f64)> = vec![
            (
                "dof_damping",
                m1.dof_damping.get(d).copied().unwrap_or(0.0),
                m2.dof_damping.get(d).copied().unwrap_or(0.0),
            ),
            (
                "dof_armature",
                m1.dof_armature.get(d).copied().unwrap_or(0.0),
                m2.dof_armature.get(d).copied().unwrap_or(0.0),
            ),
            (
                "dof_invweight0",
                m1.dof_invweight0.get(d).copied().unwrap_or(0.0),
                m2.dof_invweight0.get(d).copied().unwrap_or(0.0),
            ),
            (
                "dof_frictionloss",
                m1.dof_frictionloss.get(d).copied().unwrap_or(0.0),
                m2.dof_frictionloss.get(d).copied().unwrap_or(0.0),
            ),
        ];
        for (name, v1, v2) in fields {
            if (v1 - v2).abs() > 1e-12 {
                eprintln!("  dof[{}].{}: {:.8} vs {:.8}", d, name, v1, v2);
            }
        }
    }

    // Global options
    let globals: Vec<(&str, f64, f64)> = vec![
        ("stat_meaninertia", m1.stat_meaninertia, m2.stat_meaninertia),
        ("density", m1.density, m2.density),
        ("viscosity", m1.viscosity, m2.viscosity),
        ("o_margin", m1.o_margin, m2.o_margin),
    ];
    for (name, v1, v2) in globals {
        if (v1 - v2).abs() > 1e-12 {
            eprintln!("  {}: {:.8} vs {:.8}", name, v1, v2);
        }
    }

    // Body-level (ALL fields)
    for b in 0..m1.nbody.min(m2.nbody) {
        // Quaternion comparison
        let iq1 = m1.body_iquat.get(b).map(|q| q.as_vector().clone());
        let iq2 = m2.body_iquat.get(b).map(|q| q.as_vector().clone());
        if let (Some(q1), Some(q2)) = (&iq1, &iq2) {
            let diff = (q1 - q2).norm();
            if diff > 1e-10 {
                eprintln!(
                    "  body[{}].iquat: ({:.4},{:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4},{:.4})",
                    b, q1[0], q1[1], q1[2], q1[3], q2[0], q2[1], q2[2], q2[3]
                );
            }
        }
        let bq1 = m1.body_quat.get(b).map(|q| q.as_vector().clone());
        let bq2 = m2.body_quat.get(b).map(|q| q.as_vector().clone());
        if let (Some(q1), Some(q2)) = (&bq1, &bq2) {
            let diff = (q1 - q2).norm();
            if diff > 1e-10 {
                eprintln!("  body[{}].quat: diff={:.6}", b, diff);
            }
        }
    }

    // Joint-level (ALL fields including pos, axis)
    for j in 0..m1.njnt.min(m2.njnt) {
        let jp1 = m1.jnt_pos.get(j).copied().unwrap_or(Vector3::zeros());
        let jp2 = m2.jnt_pos.get(j).copied().unwrap_or(Vector3::zeros());
        if (jp1 - jp2).norm() > 1e-10 {
            eprintln!(
                "  jnt[{}].pos: ({:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4})",
                j, jp1.x, jp1.y, jp1.z, jp2.x, jp2.y, jp2.z
            );
        }
        let ja1 = m1.jnt_axis.get(j).copied().unwrap_or(Vector3::z());
        let ja2 = m2.jnt_axis.get(j).copied().unwrap_or(Vector3::z());
        if (ja1 - ja2).norm() > 1e-10 {
            eprintln!(
                "  jnt[{}].axis: ({:.4},{:.4},{:.4}) vs ({:.4},{:.4},{:.4})",
                j, ja1.x, ja1.y, ja1.z, ja2.x, ja2.y, ja2.z
            );
        }
        let jb1 = m1.jnt_body.get(j).copied().unwrap_or(0);
        let jb2 = m2.jnt_body.get(j).copied().unwrap_or(0);
        if jb1 != jb2 {
            eprintln!("  jnt[{}].body: {} vs {}", j, jb1, jb2);
        }
    }

    // DOF-level (body mapping)
    for d in 0..m1.nv.min(m2.nv) {
        let db1 = m1.dof_body.get(d).copied().unwrap_or(0);
        let db2 = m2.dof_body.get(d).copied().unwrap_or(0);
        if db1 != db2 {
            eprintln!("  dof[{}].body: {} vs {}", d, db1, db2);
        }
    }

    // ── EXHAUSTIVE SERIALIZATION-BASED DIFF ──────────────────────────
    {
        use std::collections::{BTreeMap, BTreeSet, HashSet};

        eprintln!("\n╔══════════════════════════════════════════════════════════════╗");
        eprintln!("║  EXHAUSTIVE DIFF: cfd_patched (STABLE) vs cfd_model (UNSTABLE)  ║");
        eprintln!("╚══════════════════════════════════════════════════════════════╝");

        let pa = &cfd_patched;
        let pb = &cfd_model;

        let mut fa: BTreeMap<String, Vec<f64>> = BTreeMap::new();
        let mut fb: BTreeMap<String, Vec<f64>> = BTreeMap::new();

        macro_rules! ins {
            ($key:expr, $va:expr, $vb:expr) => {
                fa.insert($key.to_string(), $va);
                fb.insert($key.to_string(), $vb);
            };
        }

        // === GLOBAL SCALARS ===
        ins!("timestep", vec![pa.timestep], vec![pb.timestep]);
        ins!(
            "gravity",
            vec![pa.gravity.x, pa.gravity.y, pa.gravity.z],
            vec![pb.gravity.x, pb.gravity.y, pb.gravity.z]
        );
        ins!(
            "wind",
            vec![pa.wind.x, pa.wind.y, pa.wind.z],
            vec![pb.wind.x, pb.wind.y, pb.wind.z]
        );
        ins!(
            "magnetic",
            vec![pa.magnetic.x, pa.magnetic.y, pa.magnetic.z],
            vec![pb.magnetic.x, pb.magnetic.y, pb.magnetic.z]
        );
        ins!("density", vec![pa.density], vec![pb.density]);
        ins!("viscosity", vec![pa.viscosity], vec![pb.viscosity]);
        ins!("impratio", vec![pa.impratio], vec![pb.impratio]);
        ins!("o_margin", vec![pa.o_margin], vec![pb.o_margin]);
        ins!("o_solref", pa.o_solref.to_vec(), pb.o_solref.to_vec());
        ins!("o_solimp", pa.o_solimp.to_vec(), pb.o_solimp.to_vec());
        ins!("o_friction", pa.o_friction.to_vec(), pb.o_friction.to_vec());
        ins!(
            "solver_iterations",
            vec![pa.solver_iterations as f64],
            vec![pb.solver_iterations as f64]
        );
        ins!(
            "solver_tolerance",
            vec![pa.solver_tolerance],
            vec![pb.solver_tolerance]
        );
        ins!(
            "regularization",
            vec![pa.regularization],
            vec![pb.regularization]
        );
        ins!(
            "friction_smoothing",
            vec![pa.friction_smoothing],
            vec![pb.friction_smoothing]
        );
        ins!("cone", vec![pa.cone as f64], vec![pb.cone as f64]);
        ins!(
            "stat_meaninertia",
            vec![pa.stat_meaninertia],
            vec![pb.stat_meaninertia]
        );
        ins!(
            "ls_iterations",
            vec![pa.ls_iterations as f64],
            vec![pb.ls_iterations as f64]
        );
        ins!("ls_tolerance", vec![pa.ls_tolerance], vec![pb.ls_tolerance]);
        ins!(
            "noslip_iterations",
            vec![pa.noslip_iterations as f64],
            vec![pb.noslip_iterations as f64]
        );
        ins!(
            "noslip_tolerance",
            vec![pa.noslip_tolerance],
            vec![pb.noslip_tolerance]
        );
        ins!(
            "diagapprox_bodyweight",
            vec![pa.diagapprox_bodyweight as u8 as f64],
            vec![pb.diagapprox_bodyweight as u8 as f64]
        );
        ins!(
            "disableflags",
            vec![pa.disableflags as f64],
            vec![pb.disableflags as f64]
        );
        ins!(
            "enableflags",
            vec![pa.enableflags as f64],
            vec![pb.enableflags as f64]
        );
        ins!(
            "disableactuator",
            vec![pa.disableactuator as f64],
            vec![pb.disableactuator as f64]
        );
        ins!(
            "ccd_iterations",
            vec![pa.ccd_iterations as f64],
            vec![pb.ccd_iterations as f64]
        );
        ins!(
            "ccd_tolerance",
            vec![pa.ccd_tolerance],
            vec![pb.ccd_tolerance]
        );
        ins!(
            "sdf_iterations",
            vec![pa.sdf_iterations as f64],
            vec![pb.sdf_iterations as f64]
        );
        ins!(
            "sdf_initpoints",
            vec![pa.sdf_initpoints as f64],
            vec![pb.sdf_initpoints as f64]
        );
        ins!(
            "sleep_tolerance",
            vec![pa.sleep_tolerance],
            vec![pb.sleep_tolerance]
        );
        ins!(
            "ngravcomp",
            vec![pa.ngravcomp as f64],
            vec![pb.ngravcomp as f64]
        );

        // Dimensions (informational)
        ins!("nq", vec![pa.nq as f64], vec![pb.nq as f64]);
        ins!("nv", vec![pa.nv as f64], vec![pb.nv as f64]);
        ins!("nbody", vec![pa.nbody as f64], vec![pb.nbody as f64]);
        ins!("njnt", vec![pa.njnt as f64], vec![pb.njnt as f64]);
        ins!("ngeom", vec![pa.ngeom as f64], vec![pb.ngeom as f64]);
        ins!("ntree", vec![pa.ntree as f64], vec![pb.ntree as f64]);

        // Sparse LDL
        ins!("qLD_nnz", vec![pa.qLD_nnz as f64], vec![pb.qLD_nnz as f64]);
        ins!(
            "qLD_rowadr",
            pa.qLD_rowadr.iter().map(|&x| x as f64).collect(),
            pb.qLD_rowadr.iter().map(|&x| x as f64).collect()
        );
        ins!(
            "qLD_rownnz",
            pa.qLD_rownnz.iter().map(|&x| x as f64).collect(),
            pb.qLD_rownnz.iter().map(|&x| x as f64).collect()
        );
        ins!(
            "qLD_colind",
            pa.qLD_colind.iter().map(|&x| x as f64).collect(),
            pb.qLD_colind.iter().map(|&x| x as f64).collect()
        );

        // Vectors
        ins!(
            "qpos0",
            pa.qpos0.as_slice().to_vec(),
            pb.qpos0.as_slice().to_vec()
        );
        ins!(
            "qpos_spring",
            pa.qpos_spring.clone(),
            pb.qpos_spring.clone()
        );
        ins!(
            "implicit_stiffness",
            pa.implicit_stiffness.as_slice().to_vec(),
            pb.implicit_stiffness.as_slice().to_vec()
        );
        ins!(
            "implicit_damping",
            pa.implicit_damping.as_slice().to_vec(),
            pb.implicit_damping.as_slice().to_vec()
        );
        ins!(
            "implicit_springref",
            pa.implicit_springref.as_slice().to_vec(),
            pb.implicit_springref.as_slice().to_vec()
        );

        // === PER-BODY ===
        let nb = pa.nbody.min(pb.nbody);
        for b in 0..nb {
            let p = format!("body[{}]", b);
            ins!(
                format!("{p}.parent"),
                vec![pa.body_parent[b] as f64],
                vec![pb.body_parent[b] as f64]
            );
            ins!(
                format!("{p}.rootid"),
                vec![pa.body_rootid[b] as f64],
                vec![pb.body_rootid[b] as f64]
            );
            ins!(
                format!("{p}.jnt_adr"),
                vec![pa.body_jnt_adr[b] as f64],
                vec![pb.body_jnt_adr[b] as f64]
            );
            ins!(
                format!("{p}.jnt_num"),
                vec![pa.body_jnt_num[b] as f64],
                vec![pb.body_jnt_num[b] as f64]
            );
            ins!(
                format!("{p}.dof_adr"),
                vec![pa.body_dof_adr[b] as f64],
                vec![pb.body_dof_adr[b] as f64]
            );
            ins!(
                format!("{p}.dof_num"),
                vec![pa.body_dof_num[b] as f64],
                vec![pb.body_dof_num[b] as f64]
            );
            ins!(
                format!("{p}.geom_adr"),
                vec![pa.body_geom_adr[b] as f64],
                vec![pb.body_geom_adr[b] as f64]
            );
            ins!(
                format!("{p}.geom_num"),
                vec![pa.body_geom_num[b] as f64],
                vec![pb.body_geom_num[b] as f64]
            );
            ins!(
                format!("{p}.mass"),
                vec![pa.body_mass[b]],
                vec![pb.body_mass[b]]
            );
            ins!(
                format!("{p}.inertia"),
                vec![
                    pa.body_inertia[b].x,
                    pa.body_inertia[b].y,
                    pa.body_inertia[b].z
                ],
                vec![
                    pb.body_inertia[b].x,
                    pb.body_inertia[b].y,
                    pb.body_inertia[b].z
                ]
            );
            ins!(
                format!("{p}.pos"),
                vec![pa.body_pos[b].x, pa.body_pos[b].y, pa.body_pos[b].z],
                vec![pb.body_pos[b].x, pb.body_pos[b].y, pb.body_pos[b].z]
            );
            let qa = pa.body_quat[b].as_vector();
            let qb_ = pb.body_quat[b].as_vector();
            ins!(
                format!("{p}.quat"),
                vec![qa[0], qa[1], qa[2], qa[3]],
                vec![qb_[0], qb_[1], qb_[2], qb_[3]]
            );
            ins!(
                format!("{p}.ipos"),
                vec![pa.body_ipos[b].x, pa.body_ipos[b].y, pa.body_ipos[b].z],
                vec![pb.body_ipos[b].x, pb.body_ipos[b].y, pb.body_ipos[b].z]
            );
            let iqa = pa.body_iquat[b].as_vector();
            let iqb = pb.body_iquat[b].as_vector();
            ins!(
                format!("{p}.iquat"),
                vec![iqa[0], iqa[1], iqa[2], iqa[3]],
                vec![iqb[0], iqb[1], iqb[2], iqb[3]]
            );
            ins!(
                format!("{p}.subtreemass"),
                vec![pa.body_subtreemass[b]],
                vec![pb.body_subtreemass[b]]
            );
            ins!(
                format!("{p}.gravcomp"),
                vec![pa.body_gravcomp[b]],
                vec![pb.body_gravcomp[b]]
            );
            ins!(
                format!("{p}.invweight0"),
                vec![pa.body_invweight0[b][0], pa.body_invweight0[b][1]],
                vec![pb.body_invweight0[b][0], pb.body_invweight0[b][1]]
            );
            ins!(
                format!("{p}.treeid"),
                vec![pa.body_treeid[b] as f64],
                vec![pb.body_treeid[b] as f64]
            );
        }

        // === PER-JOINT ===
        let nj = pa.njnt.min(pb.njnt);
        for j in 0..nj {
            let p = format!("jnt[{}]", j);
            ins!(
                format!("{p}.body"),
                vec![pa.jnt_body[j] as f64],
                vec![pb.jnt_body[j] as f64]
            );
            ins!(
                format!("{p}.qpos_adr"),
                vec![pa.jnt_qpos_adr[j] as f64],
                vec![pb.jnt_qpos_adr[j] as f64]
            );
            ins!(
                format!("{p}.dof_adr"),
                vec![pa.jnt_dof_adr[j] as f64],
                vec![pb.jnt_dof_adr[j] as f64]
            );
            ins!(
                format!("{p}.pos"),
                vec![pa.jnt_pos[j].x, pa.jnt_pos[j].y, pa.jnt_pos[j].z],
                vec![pb.jnt_pos[j].x, pb.jnt_pos[j].y, pb.jnt_pos[j].z]
            );
            ins!(
                format!("{p}.axis"),
                vec![pa.jnt_axis[j].x, pa.jnt_axis[j].y, pa.jnt_axis[j].z],
                vec![pb.jnt_axis[j].x, pb.jnt_axis[j].y, pb.jnt_axis[j].z]
            );
            ins!(
                format!("{p}.limited"),
                vec![pa.jnt_limited[j] as u8 as f64],
                vec![pb.jnt_limited[j] as u8 as f64]
            );
            ins!(
                format!("{p}.range"),
                vec![pa.jnt_range[j].0, pa.jnt_range[j].1],
                vec![pb.jnt_range[j].0, pb.jnt_range[j].1]
            );
            ins!(
                format!("{p}.stiffness"),
                vec![pa.jnt_stiffness[j]],
                vec![pb.jnt_stiffness[j]]
            );
            ins!(
                format!("{p}.springref"),
                vec![pa.jnt_springref[j]],
                vec![pb.jnt_springref[j]]
            );
            ins!(
                format!("{p}.damping"),
                vec![pa.jnt_damping[j]],
                vec![pb.jnt_damping[j]]
            );
            ins!(
                format!("{p}.armature"),
                vec![pa.jnt_armature[j]],
                vec![pb.jnt_armature[j]]
            );
            ins!(
                format!("{p}.solref"),
                pa.jnt_solref[j].to_vec(),
                pb.jnt_solref[j].to_vec()
            );
            ins!(
                format!("{p}.solimp"),
                pa.jnt_solimp[j].to_vec(),
                pb.jnt_solimp[j].to_vec()
            );
            ins!(
                format!("{p}.margin"),
                vec![pa.jnt_margin[j]],
                vec![pb.jnt_margin[j]]
            );
            ins!(
                format!("{p}.group"),
                vec![pa.jnt_group[j] as f64],
                vec![pb.jnt_group[j] as f64]
            );
            ins!(
                format!("{p}.actgravcomp"),
                vec![pa.jnt_actgravcomp[j] as u8 as f64],
                vec![pb.jnt_actgravcomp[j] as u8 as f64]
            );
        }

        // === PER-DOF ===
        let ndof = pa.nv.min(pb.nv);
        for d in 0..ndof {
            let p = format!("dof[{}]", d);
            ins!(
                format!("{p}.body"),
                vec![pa.dof_body[d] as f64],
                vec![pb.dof_body[d] as f64]
            );
            ins!(
                format!("{p}.jnt"),
                vec![pa.dof_jnt[d] as f64],
                vec![pb.dof_jnt[d] as f64]
            );
            ins!(
                format!("{p}.parent"),
                vec![pa.dof_parent[d].map_or(-1.0, |x| x as f64)],
                vec![pb.dof_parent[d].map_or(-1.0, |x| x as f64)]
            );
            ins!(
                format!("{p}.armature"),
                vec![pa.dof_armature[d]],
                vec![pb.dof_armature[d]]
            );
            ins!(
                format!("{p}.damping"),
                vec![pa.dof_damping[d]],
                vec![pb.dof_damping[d]]
            );
            ins!(
                format!("{p}.frictionloss"),
                vec![pa.dof_frictionloss[d]],
                vec![pb.dof_frictionloss[d]]
            );
            ins!(
                format!("{p}.solref"),
                pa.dof_solref[d].to_vec(),
                pb.dof_solref[d].to_vec()
            );
            ins!(
                format!("{p}.solimp"),
                pa.dof_solimp[d].to_vec(),
                pb.dof_solimp[d].to_vec()
            );
            ins!(
                format!("{p}.invweight0"),
                vec![pa.dof_invweight0[d]],
                vec![pb.dof_invweight0[d]]
            );
            ins!(
                format!("{p}.treeid"),
                vec![pa.dof_treeid[d] as f64],
                vec![pb.dof_treeid[d] as f64]
            );
            ins!(
                format!("{p}.length"),
                vec![pa.dof_length[d]],
                vec![pb.dof_length[d]]
            );
        }

        // === MATCHED GEOMS (by type + body) ===
        let mut used_b = HashSet::new();
        let mut geom_pairs: Vec<(usize, usize, String)> = Vec::new();
        for ga in 0..pa.ngeom {
            for gb in 0..pb.ngeom {
                if used_b.contains(&gb) {
                    continue;
                }
                if pb.geom_type[gb] == pa.geom_type[ga] && pb.geom_body[gb] == pa.geom_body[ga] {
                    geom_pairs.push((
                        ga,
                        gb,
                        format!("{:?}_body{}", pa.geom_type[ga], pa.geom_body[ga]),
                    ));
                    used_b.insert(gb);
                    break;
                }
            }
        }

        for (ga, gb, label) in &geom_pairs {
            let p = format!("geom[{}]", label);
            ins!(
                format!("{p}.pos"),
                vec![pa.geom_pos[*ga].x, pa.geom_pos[*ga].y, pa.geom_pos[*ga].z],
                vec![pb.geom_pos[*gb].x, pb.geom_pos[*gb].y, pb.geom_pos[*gb].z]
            );
            let gqa = pa.geom_quat[*ga].as_vector();
            let gqb = pb.geom_quat[*gb].as_vector();
            ins!(
                format!("{p}.quat"),
                vec![gqa[0], gqa[1], gqa[2], gqa[3]],
                vec![gqb[0], gqb[1], gqb[2], gqb[3]]
            );
            ins!(
                format!("{p}.size"),
                vec![
                    pa.geom_size[*ga].x,
                    pa.geom_size[*ga].y,
                    pa.geom_size[*ga].z
                ],
                vec![
                    pb.geom_size[*gb].x,
                    pb.geom_size[*gb].y,
                    pb.geom_size[*gb].z
                ]
            );
            ins!(
                format!("{p}.friction"),
                vec![
                    pa.geom_friction[*ga].x,
                    pa.geom_friction[*ga].y,
                    pa.geom_friction[*ga].z
                ],
                vec![
                    pb.geom_friction[*gb].x,
                    pb.geom_friction[*gb].y,
                    pb.geom_friction[*gb].z
                ]
            );
            ins!(
                format!("{p}.condim"),
                vec![pa.geom_condim[*ga] as f64],
                vec![pb.geom_condim[*gb] as f64]
            );
            ins!(
                format!("{p}.contype"),
                vec![pa.geom_contype[*ga] as f64],
                vec![pb.geom_contype[*gb] as f64]
            );
            ins!(
                format!("{p}.conaffinity"),
                vec![pa.geom_conaffinity[*ga] as f64],
                vec![pb.geom_conaffinity[*gb] as f64]
            );
            ins!(
                format!("{p}.margin"),
                vec![pa.geom_margin[*ga]],
                vec![pb.geom_margin[*gb]]
            );
            ins!(
                format!("{p}.gap"),
                vec![pa.geom_gap[*ga]],
                vec![pb.geom_gap[*gb]]
            );
            ins!(
                format!("{p}.priority"),
                vec![pa.geom_priority[*ga] as f64],
                vec![pb.geom_priority[*gb] as f64]
            );
            ins!(
                format!("{p}.solmix"),
                vec![pa.geom_solmix[*ga]],
                vec![pb.geom_solmix[*gb]]
            );
            ins!(
                format!("{p}.solimp"),
                pa.geom_solimp[*ga].to_vec(),
                pb.geom_solimp[*gb].to_vec()
            );
            ins!(
                format!("{p}.solref"),
                pa.geom_solref[*ga].to_vec(),
                pb.geom_solref[*gb].to_vec()
            );
            ins!(
                format!("{p}.rbound"),
                vec![pa.geom_rbound[*ga]],
                vec![pb.geom_rbound[*gb]]
            );
            ins!(
                format!("{p}.aabb"),
                pa.geom_aabb[*ga].to_vec(),
                pb.geom_aabb[*gb].to_vec()
            );
            ins!(
                format!("{p}.group"),
                vec![pa.geom_group[*ga] as f64],
                vec![pb.geom_group[*gb] as f64]
            );
            ins!(
                format!("{p}.rgba"),
                pa.geom_rgba[*ga].to_vec(),
                pb.geom_rgba[*gb].to_vec()
            );
            ins!(
                format!("{p}.fluid"),
                pa.geom_fluid[*ga].to_vec(),
                pb.geom_fluid[*gb].to_vec()
            );
        }

        // === TREE ===
        let nt = pa.ntree.min(pb.ntree);
        for t in 0..nt {
            let p = format!("tree[{}]", t);
            ins!(
                format!("{p}.body_adr"),
                vec![pa.tree_body_adr[t] as f64],
                vec![pb.tree_body_adr[t] as f64]
            );
            ins!(
                format!("{p}.body_num"),
                vec![pa.tree_body_num[t] as f64],
                vec![pb.tree_body_num[t] as f64]
            );
            ins!(
                format!("{p}.dof_adr"),
                vec![pa.tree_dof_adr[t] as f64],
                vec![pb.tree_dof_adr[t] as f64]
            );
            ins!(
                format!("{p}.dof_num"),
                vec![pa.tree_dof_num[t] as f64],
                vec![pb.tree_dof_num[t] as f64]
            );
        }

        // === AUTO-DIFF ===
        let all_keys: BTreeSet<String> = fa.keys().chain(fb.keys()).cloned().collect();
        let mut diff_count = 0usize;
        let mut diff_fields: Vec<String> = Vec::new();

        for key in &all_keys {
            match (fa.get(key), fb.get(key)) {
                (Some(va), Some(vb)) => {
                    if va.len() == vb.len() {
                        for (i, (a, b)) in va.iter().zip(vb.iter()).enumerate() {
                            if (a - b).abs() > 1e-12 {
                                if va.len() == 1 {
                                    eprintln!(
                                        "  DIFF {}: {:.10} vs {:.10} (Δ={:.2e})",
                                        key,
                                        a,
                                        b,
                                        (a - b).abs()
                                    );
                                } else {
                                    eprintln!(
                                        "  DIFF {}[{}]: {:.10} vs {:.10} (Δ={:.2e})",
                                        key,
                                        i,
                                        a,
                                        b,
                                        (a - b).abs()
                                    );
                                }
                                diff_count += 1;
                                if !diff_fields.contains(key) {
                                    diff_fields.push(key.clone());
                                }
                            }
                        }
                    } else {
                        eprintln!("  DIFF {}: len {} vs {}", key, va.len(), vb.len());
                        diff_count += 1;
                        diff_fields.push(key.clone());
                    }
                }
                (Some(_), None) => {
                    eprintln!("  DIFF {}: in patched only", key);
                    diff_count += 1;
                    diff_fields.push(key.clone());
                }
                (None, Some(_)) => {
                    eprintln!("  DIFF {}: in cfd only", key);
                    diff_count += 1;
                    diff_fields.push(key.clone());
                }
                (None, None) => unreachable!(),
            }
        }

        // Enum fields (non-numeric comparison)
        if format!("{:?}", pa.integrator) != format!("{:?}", pb.integrator) {
            eprintln!(
                "  DIFF integrator: {:?} vs {:?}",
                pa.integrator, pb.integrator
            );
            diff_count += 1;
            diff_fields.push("integrator".into());
        }
        if format!("{:?}", pa.solver_type) != format!("{:?}", pb.solver_type) {
            eprintln!(
                "  DIFF solver_type: {:?} vs {:?}",
                pa.solver_type, pb.solver_type
            );
            diff_count += 1;
            diff_fields.push("solver_type".into());
        }
        for j in 0..nj {
            if format!("{:?}", pa.jnt_type[j]) != format!("{:?}", pb.jnt_type[j]) {
                eprintln!(
                    "  DIFF jnt[{}].type: {:?} vs {:?}",
                    j, pa.jnt_type[j], pb.jnt_type[j]
                );
                diff_count += 1;
                diff_fields.push(format!("jnt[{}].type", j));
            }
        }

        eprintln!("\n  ══════════════════════════════════════════");
        eprintln!("  Total fields checked: {}", all_keys.len() + 3); // +3 for enum fields
        eprintln!("  Total diff entries: {}", diff_count);
        eprintln!("  Diff fields: {:?}", diff_fields);
        eprintln!("  ══════════════════════════════════════════");
    }
}
