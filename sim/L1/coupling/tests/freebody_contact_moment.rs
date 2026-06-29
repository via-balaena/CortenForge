//! Free-body off-COM contact **moment** gate (Layer-2 keystone, forward physics).
//!
//! [`StaggeredCoupling::step`] routes the soft contact reaction onto a *free* rigid
//! body. Before [`StaggeredCoupling::with_contact_moment`] it routed only the linear
//! reaction `[0; f]`, so an **off-centre** strike was wrongly torque-free. With the
//! moment on it routes the full wrench `[τ; f]`, `τ = Σ (rᵢ − c) × (−gᵢ)` about the
//! body COM `c` — the same moment definition [`StaggeredCoupling::step_articulated`]
//! already uses, reduced here from `step`'s own contact readouts (no second eval).
//!
//! ## The exact contract these gates check
//! A flat box platen, **pre-penetrated and released from rest** (`qvel = 0`,
//! orientation `R = I`), with its geom centred in the body so the COM is the body origin
//! (the free-joint mass matrix is then block-diagonal — no linear↔angular coupling). The
//! infinite-plane contact is **invariant to the platen's x,y position** (it enters no
//! penetration, force, or soft-solve term — only the rigid `z` matters), so two scenes
//! whose COM differs only in x see a **bit-identical** contact: same contact points `rᵢ`,
//! same forces `gᵢ`. The *only* thing that differs is the COM `c` in the moment arm, so
//! the messy center-of-pressure cancels in the difference and the moment-arm law is exact:
//!
//! ```text
//! τ_y(c_A) − τ_y(c_B) = [(c_B − c_A) × Σfᵢ]_y = (c_Ax − c_Bx)·F_z
//! ⇒ ω_y(+δ) − ω_y(−δ) = 2·δ·F_z·dt / I_yy   (first step from rest: no Coriolis)
//! ```
//!
//! with `F_z = −force_on_soft.z` the reaction `step` returns and `I_yy = m/12·(lx²+lz²)`
//! the box's principal inertia (full extents). This isolates the moment **arm** (the
//! `(rᵢ − c)` law, about the COM, integrated by the body's own inertia) with no dependence
//! on where the discrete contact pressure actually centres — every quantity is a public
//! output, so no extra surface is needed.
//!
//! (The absolute spin carries a small residual even when the COM sits over the geometric
//! centroid — the corner-origin CFK tet mesh is not mirror-symmetric, so its discrete
//! contact pressure centres a few mm off the geometric centroid. That is a real, correct
//! consequence of the asymmetric discretisation, not a routing bug; it cancels in the
//! difference above, and a deliberate offset `δ` dominates it.)
//!
//! Why opt-in: turning the moment on changes the rotational physics of every free-body
//! scene, and the flat-tuned keystone fixtures are geometrically off-centre (corner-origin
//! block centroid `(edge/2, edge/2)` ~`edge/2` from the platen COM at the origin), so an
//! always-on moment injects a large undamped tipping torque (`rigid_damping` damps only
//! the linear axis) — a spike measured `ω ≈ 358 rad/s` on the canonical scene. These gates
//! therefore re-centre the platen over the block centroid. See `with_contact_moment`.

#![allow(
    // A missing/malformed fixture (MJCF load, body index) surfaces as a test panic —
    // the canonical fixture idiom in this workspace's integration tests.
    clippy::expect_used
)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// Soft block: corner-origin cube spanning [0, EDGE]³, contact centroid (EDGE/2, EDGE/2).
const EDGE: f64 = 0.1;
const CENTROID: f64 = EDGE / 2.0; // 0.05

// Platen box: half-extents (0.06, 0.06, 0.005), mass 0.2 — the canonical keystone platen.
const MASS: f64 = 0.2;
const HALF_X: f64 = 0.06;
const HALF_Z: f64 = 0.005;
const CLEARANCE: f64 = HALF_Z; // contact plane at COM_z − half-thickness
const DT: f64 = 1.0e-3;
const PEN: f64 = 1.0e-4; // pre-penetration into the block top (gives a finite first-step force)
const DELTA: f64 = 0.02; // COM-x offset from the block centroid

/// Principal inertia `I_yy = m/12·(lx² + lz²)` of the platen box (full extents),
/// matching `sim-mjcf`'s box-geom inertia (`builder/geom.rs`).
fn platen_iyy() -> f64 {
    let lx = 2.0 * HALF_X;
    let lz = 2.0 * HALF_Z;
    MASS / 12.0 * (lx * lx + lz * lz)
}

/// A flat platen whose COM sits at `(com_x, CENTROID)`, pre-penetrated [`PEN`] metres into
/// the block top (`z = EDGE`) and at rest. The geom is centred in the body, so the COM is
/// the body origin. `moment` toggles [`StaggeredCoupling::with_contact_moment`].
fn make_coupling(com_x: f64, moment: bool) -> StaggeredCoupling {
    // plane_height = COM_z − CLEARANCE sits PEN below the block top (z = EDGE).
    let com_z = EDGE + CLEARANCE - PEN;
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="{DT}"/>
  <worldbody>
    <body name="platen" pos="{com_x} {CENTROID} {com_z}">
      <freejoint/>
      <geom type="box" size="{HALF_X} {HALF_X} {HALF_Z}" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, /* body */ 1, CLEARANCE, /* n_per_edge */ 4, EDGE,
        /* mu */ 3.0e4, DT, /* kappa */ 3.0e4, /* d_hat */ 1.0e-2,
        /* rigid_damping */ 8.0,
    )
    .with_contact_moment(moment)
}

/// `(F_z reaction, body angular velocity)` after one step from rest, for a platen whose
/// COM is offset `com_x` in x. The contact (and thus `F_z`) is identical for any `com_x`.
fn first_step(com_x: f64) -> (f64, [f64; 3]) {
    let mut c = make_coupling(com_x, true);
    let step = c.step();
    let w = &c.data().qvel;
    (-step.force_on_soft.z, [w[3], w[4], w[5]])
}

/// The headline contract: the moment-**arm** law is exact. Two platens offset `±δ` in x see
/// a bit-identical contact, so the difference in first-step `ω_y` is exactly `2δ·F_z·dt/I_yy`
/// — independent of where the discrete contact pressure actually centres. Also pins the
/// premise (`F_z` is COM-x-invariant), the tip **sense** (a `+x` offset gives `+ω_y`), and the
/// absence of a spurious z-torque — all from the one fixture pair, no extra FEM solves.
#[test]
fn offcentre_moment_arm_law_is_exact() {
    let (f_pos, w_pos) = first_step(CENTROID + DELTA);
    let (f_neg, w_neg) = first_step(CENTROID - DELTA);

    // Premise: the infinite-plane contact is invariant to the COM's x ⇒ bit-identical F_z.
    assert!(f_pos > 0.0, "contact should be engaged, got F_z = {f_pos}");
    assert_eq!(
        f_pos, f_neg,
        "plane contact must be COM-x-invariant: {f_pos} vs {f_neg}"
    );

    // Moment-arm law: ω_y(+δ) − ω_y(−δ) = 2·δ·F_z·dt/I_yy, exact (the cop offset cancels).
    let measured = w_pos[1] - w_neg[1];
    let predicted = 2.0 * DELTA * f_pos * DT / platen_iyy();
    let rel = (measured - predicted).abs() / predicted.abs();
    assert!(
        rel < 1.0e-12,
        "moment-arm law: Δω_y = {measured} vs 2δ·F_z·dt/I_yy = {predicted} (rel {rel}); F_z = {f_pos} N"
    );

    // Sense: a `+x`-offset COM under an upward reaction tips one way, `−x` the other. The
    // deliberate offset δ dominates the small mesh-asymmetry residual, so the signs are clean.
    assert!(
        w_pos[1] > 0.0 && w_neg[1] < 0.0,
        "offset should set the tip direction: ω_y(+δ) = {}, ω_y(−δ) = {}",
        w_pos[1],
        w_neg[1]
    );

    // Vertical reaction ⇒ no torque about z, exactly, at both offsets.
    assert_eq!(
        w_pos[2], 0.0,
        "no spurious z-torque (+δ): ω_z = {}",
        w_pos[2]
    );
    assert_eq!(
        w_neg[2], 0.0,
        "no spurious z-torque (−δ): ω_z = {}",
        w_neg[2]
    );
}

/// Default (moment off) routes only the linear reaction: the same off-centre scene that
/// spins with the moment on stays *exactly* torque-free — the flag is precisely what
/// introduces the spin, and the byte-identical default keeps the keystone suite unperturbed.
#[test]
fn moment_off_routes_no_torque() {
    let mut c = make_coupling(CENTROID + DELTA, false); // moment OFF (default)
    let step = c.step();
    assert!(step.force_on_soft.z < 0.0, "contact should be engaged");
    let w = &c.data().qvel;
    // No angular force is applied (sf[0..3] stays zero), so the only angular velocity is
    // machine noise from the free-joint solve — ~1e-16, vs the ~1e2 rad/s the moment drives.
    let max_ang = w[3].abs().max(w[4].abs()).max(w[5].abs());
    assert!(
        max_ang < 1.0e-12,
        "with the moment off, an off-centre strike must stay torque-free, got max|ω| = {max_ang} \
         (ω = ({}, {}, {}))",
        w[3],
        w[4],
        w[5]
    );
}
