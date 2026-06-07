//! Emit MJCF for the knee subgraph at two fidelities.
//!
//! * [`emit_frozen_hinge`] — the S0 baseline: a bare 1-DOF hinge with the
//!   coupled tibial translation frozen at θ=0 (risk R3) and moving points frozen
//!   (O2). S0 showed this misses the 5 mm budget for all four muscles.
//! * [`emit_coupled_knee`] — the S1 model: the tibia rides two coupled slide
//!   DOFs (`knee_tx`, `knee_ty`) plus the hinge (`knee`), and each patella
//!   `MovingPathPoint` becomes a small "patella" body under the tibia carrying
//!   three slide DOFs so its site moves with the knee angle. All coupled DOFs are
//!   driven kinematically by the caller (see `lib::coupled_moment_arm`);
//!   equality-joint coupling for dynamic actuation is a G2 add.
//!
//! With no wrap objects, the muscles are straight multi-site spatial tendons.

use crate::osim::{Kind, Spline, Subgraph};
use std::fmt::Write as _;

const TINY: &str = r#"<inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>"#;

/// A patella body: three slide DOFs whose positions trace a `MovingPathPoint`'s
/// location splines (relative to the tibia). The caller drives `jx/jy/jz` qpos
/// with `sx/sy/sz` to place the muscle's moving site.
#[derive(Clone)]
pub struct Patella {
    pub jx: String,
    pub jy: String,
    pub jz: String,
    pub sx: Spline,
    pub sy: Spline,
    pub sz: Spline,
}

/// The emitted MJCF plus the muscle order and any patella bodies.
pub struct Emitted {
    pub mjcf: String,
    /// Muscle names, in order — each is also its tendon's name in the MJCF.
    pub muscles: Vec<String>,
    /// Coupled patella bodies (empty for the frozen-hinge emitter).
    pub patellae: Vec<Patella>,
}

/// S0 baseline: bare frozen hinge. Every point active at θ=0 becomes a fixed
/// site; moving points are frozen at their θ=0 location.
pub fn emit_frozen_hinge(sub: &Subgraph) -> Emitted {
    let (mut pelvis, mut femur, mut tibia, mut tendons, mut muscles) = (
        String::new(),
        String::new(),
        String::new(),
        String::new(),
        Vec::new(),
    );
    for m in &sub.muscles {
        let mut seq = String::new();
        for (i, p) in m.path.iter().filter(|p| p.active(0.0)).enumerate() {
            let site = format!("{}_{i}", m.name);
            let loc = p.location_at(0.0, true);
            let line = format!(
                "        <site name=\"{site}\" pos=\"{} {} {}\"/>\n",
                loc.x, loc.y, loc.z
            );
            push_fixed(&p.body, &line, &mut pelvis, &mut femur, &mut tibia);
            let _ = writeln!(seq, "      <site site=\"{site}\"/>");
        }
        let _ = write!(
            tendons,
            "    <spatial name=\"{}\">\n{seq}    </spatial>\n",
            m.name
        );
        muscles.push(m.name.clone());
    }
    let hip = sub.hip_in_pelvis;
    let t0 = (
        sub.knee.tx.eval(0.0),
        sub.knee.ty.eval(0.0),
        sub.knee.tz.eval(0.0),
    );
    let ax = sub.knee.flexion_axis;
    let mjcf = format!(
        r#"<mujoco>
  <worldbody>
    <body name="pelvis">
{pelvis}      <body name="femur_r" pos="{hx} {hy} {hz}">
{femur}        <body name="tibia_r" pos="{t0x} {t0y} {t0z}">
          <joint name="knee" type="hinge" axis="{ax} {ay} {az}"/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
{tibia}        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
{tendons}  </tendon>
</mujoco>
"#,
        hx = hip.x,
        hy = hip.y,
        hz = hip.z,
        t0x = t0.0,
        t0y = t0.1,
        t0z = t0.2,
        ax = ax.x,
        ay = ax.y,
        az = ax.z,
    );
    Emitted {
        mjcf,
        muscles,
        patellae: Vec::new(),
    }
}

/// S1 model: coupled tibial translation (slides) + a coupled patella body per
/// `MovingPathPoint` + conditional points dropped. Reproduces the OpenSim knee
/// moment arms within 5 mm for all four target muscles.
pub fn emit_coupled_knee(sub: &Subgraph) -> Emitted {
    let (mut pelvis, mut femur, mut tibia_fixed, mut patella_bodies) =
        (String::new(), String::new(), String::new(), String::new());
    let (mut tendons, mut muscles, mut patellae) = (String::new(), Vec::new(), Vec::new());

    for m in &sub.muscles {
        let mut seq = String::new();
        // Drop conditional points: a static tendon can't toggle membership, so
        // always-including one routes the path through it spuriously where it
        // should be inactive. A one-off probe showed dropping beats always-
        // including for semimem_r (2.5 vs 7.8 mm RMSE) — but note this DROPS a
        // point (semimem_r-P2) that is genuinely active in [0,-32°] of the sweep,
        // so it's a real structural approximation, not a free lunch. Proper
        // handling (membership toggle / wrap geom) is future work.
        let active = |p: &&crate::osim::PathPoint| {
            p.active(0.0) && !matches!(p.kind, Kind::Conditional { .. })
        };
        for (i, p) in m.path.iter().filter(active).enumerate() {
            let site = format!("{}_{i}", m.name);
            match &p.kind {
                Kind::Moving(s) => {
                    let (jx, jy, jz) = (
                        format!("{site}_x"),
                        format!("{site}_y"),
                        format!("{site}_z"),
                    );
                    let _ = write!(
                        patella_bodies,
                        "          <body name=\"{site}_pat\">\n\
                         \x20           <joint name=\"{jx}\" type=\"slide\" axis=\"1 0 0\"/>\n\
                         \x20           <joint name=\"{jy}\" type=\"slide\" axis=\"0 1 0\"/>\n\
                         \x20           <joint name=\"{jz}\" type=\"slide\" axis=\"0 0 1\"/>\n\
                         \x20           {TINY}\n\
                         \x20           <site name=\"{site}\" pos=\"0 0 0\"/>\n\
                         \x20         </body>\n"
                    );
                    patellae.push(Patella {
                        jx,
                        jy,
                        jz,
                        sx: s.x.clone(),
                        sy: s.y.clone(),
                        sz: s.z.clone(),
                    });
                }
                _ => {
                    let loc = p.location_at(0.0, true);
                    let line = format!(
                        "            <site name=\"{site}\" pos=\"{} {} {}\"/>\n",
                        loc.x, loc.y, loc.z
                    );
                    push_fixed(&p.body, &line, &mut pelvis, &mut femur, &mut tibia_fixed);
                }
            }
            let _ = writeln!(seq, "      <site site=\"{site}\"/>");
        }
        let _ = write!(
            tendons,
            "    <spatial name=\"{}\">\n{seq}    </spatial>\n",
            m.name
        );
        muscles.push(m.name.clone());
    }

    let hip = sub.hip_in_pelvis;
    let ax = sub.knee.flexion_axis;
    let mjcf = format!(
        r#"<mujoco>
  <worldbody>
    <body name="pelvis">
{pelvis}      <body name="femur_r" pos="{hx} {hy} {hz}">
{femur}        <body name="knee_tx">
          <joint name="knee_tx" type="slide" axis="1 0 0"/>
          {TINY}
          <body name="knee_ty">
            <joint name="knee_ty" type="slide" axis="0 1 0"/>
            {TINY}
            <body name="tibia_r">
              <joint name="knee" type="hinge" axis="{ax} {ay} {az}"/>
              <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
{tibia_fixed}{patella_bodies}            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
{tendons}  </tendon>
</mujoco>
"#,
        hx = hip.x,
        hy = hip.y,
        hz = hip.z,
        ax = ax.x,
        ay = ax.y,
        az = ax.z,
    );
    Emitted {
        mjcf,
        muscles,
        patellae,
    }
}

fn push_fixed(body: &str, line: &str, pelvis: &mut String, femur: &mut String, tibia: &mut String) {
    match body {
        "pelvis" | "ground" => pelvis.push_str(line),
        "femur_r" => femur.push_str(line),
        "tibia_r" => tibia.push_str(line),
        // S0 emits only the knee subgraph; any other body is a caller bug.
        other => panic!("S0 knee-only model has no body '{other}'"),
    }
}
