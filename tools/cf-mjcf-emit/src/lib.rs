//! cf-mjcf-emit — lower a [`cf_msk_lib::Model`] (a source-agnostic kinematic tree
//! of `CustomJoint` transform axes + straight-segment muscle paths) into MJCF that
//! our sim engine imports.
//!
//! This is the general emitter the leg-region cutover (recon A1) promised: it
//! replaces `cf-osim`'s bespoke knee emitter with one that walks an arbitrary
//! `Body` tree and classifies each transform axis:
//!
//! * **free DOF** — a rotation/translation with a `LinearFunction` (e.g. the knee
//!   flexion hinge): a free MJCF joint whose coordinate the caller sets directly.
//! * **coupled DOF** — a translation with a `SimmSpline` (e.g. the tibial
//!   roll-glide): an interposed wrapper *body* carrying a `slide` joint, driven
//!   kinematically by the caller from the spline (the validated S1 representation,
//!   the only way to reproduce the coupled knee without an equality constraint).
//! * **constant** — folded into the child body's `pos` (translation) or skipped
//!   when it is an identity rotation (gait2392's zero rotation2/3 and the zero
//!   `tz`).
//!
//! Each `MovingPathPoint` (the patella emulation) becomes a small body carrying
//! three coupled slide DOFs; `ConditionalPathPoint`s are dropped (a static tendon
//! can't toggle membership — the one G1 structural approximation, called out).
//!
//! Every emitted non-fixed joint is reported in [`Emitted::driven`] as a
//! [`DrivenJoint`] — its MJCF joint name, the model coordinate that drives it, and
//! the function mapping coordinate → joint value. So a free DOF and a coupled DOF
//! are the *same* concept (the hinge is just a `Linear { coeff: 1 }`): to pose the
//! model at a set of coordinate values, set each driven joint's qpos to
//! `function.eval(coordinate)`. [`Emitted::qpos_targets`] does that mapping.
//!
//! **Checkpoint** (`tests/general_emit.rs`). The emitted MJCF, loaded and swept,
//! reproduces the OpenSim oracle's knee moment arms within the S1 **5 mm** gate for
//! all four muscles — the same gate the bespoke `emit_coupled_knee` met (the
//! residual is the dropped-conditional approximation). `bifemlh_r` (no
//! conditional/moving points) matches the oracle to ~machine precision, anchoring
//! that the emit geometry + coupled driving are exact. Byte stability is guarded by
//! a committed snapshot of *this* emitter's canonical output
//! (`tests/assets/knee_ref.xml`). The underlying FK is the oracle itself, anchored
//! against real OpenSim 4.6 in `cf-osim`'s `opensim_cross_check`.

use cf_msk_lib::ir::TransformFn;
use cf_msk_lib::{Body, CanonicalSource, Kind, Model, ParamSource, Spline, TransformAxis, realize};
use nalgebra::Vector3;
use std::collections::HashMap;
use std::fmt::Write as _;

const TINY: &str = r#"<inertial pos="0 0 0" mass="0.01" diaginertia="0.001 0.001 0.001"/>"#;
const MAIN_INERTIAL: &str = r#"<inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>"#;

/// A joint whose position is driven kinematically by a model coordinate:
/// `qpos = function.eval(coordinate_value)`. Covers both the free DOFs (a `Linear`
/// function, typically `coeff = 1` so qpos = the coordinate) and the coupled DOFs
/// (a `Spline`). The caller drives these to pose the model (see
/// [`Emitted::qpos_targets`]).
#[derive(Debug, Clone)]
pub struct DrivenJoint {
    pub joint: String,
    pub coordinate: String,
    pub function: TransformFn,
}

/// The emitted MJCF plus the muscle order and the driven joints.
pub struct Emitted {
    pub mjcf: String,
    /// Muscle names, in order — each is also its tendon's name in the MJCF.
    pub muscles: Vec<String>,
    /// Every non-fixed joint and how a coordinate drives it.
    pub driven: Vec<DrivenJoint>,
}

impl Emitted {
    /// The `(joint name, qpos value)` pairs that pose the model at `coords` — for
    /// each driven joint, `function.eval(coords[coordinate])` (absent coordinate =
    /// 0). The caller maps joint names to `jnt_qpos_adr` and writes `qpos`. Keeps
    /// this crate free of a `sim-core` dependency (the loaded-model wiring lives in
    /// the caller).
    pub fn qpos_targets(&self, coords: &HashMap<String, f64>) -> Vec<(String, f64)> {
        self.driven
            .iter()
            .map(|d| {
                let q = coords.get(&d.coordinate).copied().unwrap_or(0.0);
                (d.joint.clone(), d.function.eval(q))
            })
            .collect()
    }
}

/// Emit MJCF for `model`: walk the body tree and lower each joint + muscle path.
pub fn emit(model: &Model) -> Emitted {
    // Assign each active path point a site, grouped by the body it rides. A
    // `MovingPathPoint` becomes a patella sub-body; conditionals are dropped.
    let mut sites: HashMap<&str, Vec<SiteSpec>> = HashMap::new();
    let mut patellae: HashMap<&str, Vec<PatellaSpec>> = HashMap::new();
    let mut tendons: Vec<(String, Vec<String>)> = Vec::new();
    for m in &model.muscles {
        let mut seq = Vec::new();
        let active = m
            .path
            .iter()
            .filter(|p| !matches!(p.kind, Kind::Conditional { .. }));
        for (i, p) in active.enumerate() {
            // A path point must ride a body in the tree, else its site is never
            // emitted (only walked bodies write sites) yet the tendon still
            // references it → a dangling-site MJCF load failure. Fail loudly here
            // instead, matching the parser/classify convention.
            assert!(
                model.index_of(&p.body).is_some(),
                "muscle '{}' point '{}' rides unknown body '{}'",
                m.name,
                p.name,
                p.body
            );
            let site = format!("{}_{i}", m.name);
            match &p.kind {
                Kind::Moving(s) => patellae
                    .entry(p.body.as_str())
                    .or_default()
                    .push(PatellaSpec {
                        site: site.clone(),
                        coordinate: s.coordinate.clone(),
                        x: s.x.clone(),
                        y: s.y.clone(),
                        z: s.z.clone(),
                    }),
                _ => sites.entry(p.body.as_str()).or_default().push(SiteSpec {
                    name: site.clone(),
                    location: p.location,
                }),
            }
            seq.push(site);
        }
        tendons.push((m.name.clone(), seq));
    }

    let mut driven = Vec::new();
    let mut body_xml = String::new();
    let root = model
        .bodies
        .iter()
        .position(|b| b.parent.is_none())
        // a leg chain always has a root (pelvis).
        .unwrap_or_else(|| panic!("model has no root body"));
    let ctx = EmitCtx {
        model,
        sites,
        patellae,
    };
    emit_body(&ctx, root, 2, &mut body_xml, &mut driven);

    let mut tendon_xml = String::new();
    for (name, seq) in &tendons {
        let _ = writeln!(tendon_xml, "    <spatial name=\"{name}\">");
        for site in seq {
            let _ = writeln!(tendon_xml, "      <site site=\"{site}\"/>");
        }
        let _ = writeln!(tendon_xml, "    </spatial>");
    }

    let mjcf = format!(
        "<mujoco>\n  <worldbody>\n{body_xml}  </worldbody>\n  <tendon>\n{tendon_xml}  </tendon>\n</mujoco>\n"
    );
    Emitted {
        mjcf,
        muscles: model.muscles.iter().map(|m| m.name.clone()).collect(),
        driven,
    }
}

/// Morph `template` with `source`'s parameters and emit — the full
/// `ParamSource → realize → emit` path.
pub fn build(template: &Model, source: &dyn ParamSource) -> Emitted {
    emit(&realize(template, &source.params(template)))
}

/// The canonical (no-scan) body: [`build`] with [`CanonicalSource`]. The headline
/// builder-first artifact — a simulatable knee that needs no scan to exist.
pub fn build_canonical(template: &Model) -> Emitted {
    build(template, &CanonicalSource)
}

// --- internals -------------------------------------------------------------

/// A fixed muscle site on a body.
struct SiteSpec {
    name: String,
    location: Vector3<f64>,
}

/// A `MovingPathPoint`: a patella sub-body with three coupled slide DOFs tracing
/// the per-axis location splines, driven by `coordinate` (from the IR).
struct PatellaSpec {
    site: String,
    coordinate: String,
    x: Spline,
    y: Spline,
    z: Spline,
}

/// Read-only context threaded through the recursive body walk: the tree and the
/// sites + patellae grouped by owning body. The accumulators (`out`, `driven`)
/// stay explicit `&mut` parameters.
struct EmitCtx<'a> {
    model: &'a Model,
    sites: HashMap<&'a str, Vec<SiteSpec>>,
    patellae: HashMap<&'a str, Vec<PatellaSpec>>,
}

/// The coordinate that drives moving path points (and any free DOF without its own
/// Classify a joint's axes into folded constants, coupled translations, and free
/// DOFs (in declaration order). Panics on shapes A1 does not handle.
fn classify(body: &Body) -> (Vector3<f64>, Vec<&TransformAxis>, Vec<&TransformAxis>) {
    let mut const_trans = Vector3::zeros();
    let mut coupled = Vec::new();
    let mut free = Vec::new();
    for ax in &body.joint {
        match (&ax.function, ax.rotation) {
            // Constant translation → fold into the body's static offset.
            (TransformFn::Constant(v), false) => const_trans += ax.axis * *v,
            // Constant rotation must be identity (gait2392's zero rotation2/3).
            (TransformFn::Constant(v), true) => assert!(
                v.abs() < 1e-12,
                "non-zero constant rotation (fixed orientation) is unsupported"
            ),
            // A zero-gain linear axis is a no-op.
            (TransformFn::Linear { coeff }, _) if coeff.abs() < 1e-12 => {}
            // Free DOF (a real coordinate).
            (TransformFn::Linear { .. }, _) => free.push(ax),
            // Coupled translation (roll-glide) → wrapper slide body.
            (TransformFn::Spline(_), false) => coupled.push(ax),
            // A coupled rotation (spline hinge) would need an equality constraint.
            (TransformFn::Spline(_), true) => {
                panic!("coupled rotation (spline-driven hinge) is unsupported in A1")
            }
        }
    }
    (const_trans, coupled, free)
}

fn emit_body(
    ctx: &EmitCtx,
    idx: usize,
    depth: usize,
    out: &mut String,
    driven: &mut Vec<DrivenJoint>,
) {
    let b = &ctx.model.bodies[idx];
    let (const_trans, coupled, free) = classify(b);
    let pos = b.location_in_parent + const_trans;

    // Interposed wrapper bodies for the coupled translation slides (outermost
    // carries the joint-frame offset; the rest sit at the origin).
    let mut d = depth;
    for (k, ax) in coupled.iter().enumerate() {
        let wpos = if k == 0 { pos } else { Vector3::zeros() };
        let jname = format!("{}_t{}", b.name, axis_letter(ax.axis));
        let _ = writeln!(
            out,
            "{}<body name=\"{jname}\" pos=\"{}\">",
            ind(d),
            v3(wpos)
        );
        let _ = writeln!(
            out,
            "{}<joint name=\"{jname}\" type=\"slide\" axis=\"{}\"/>",
            ind(d + 1),
            v3(ax.axis)
        );
        let _ = writeln!(out, "{}{TINY}", ind(d + 1));
        driven.push(DrivenJoint {
            joint: jname,
            coordinate: ax.coordinate.clone(),
            function: ax.function.clone(),
        });
        d += 1;
    }

    // The real (anatomical) body.
    let real_pos = if coupled.is_empty() {
        pos
    } else {
        Vector3::zeros()
    };
    let _ = writeln!(
        out,
        "{}<body name=\"{}\" pos=\"{}\">",
        ind(d),
        b.name,
        v3(real_pos)
    );
    let cd = d + 1; // child indent inside the real body

    for ax in &free {
        let jtype = if ax.rotation { "hinge" } else { "slide" };
        // Name the free DOF after its coordinate (the knee hinge → "knee_angle_r").
        let jname = if ax.coordinate.is_empty() {
            format!("{}_free", b.name)
        } else {
            ax.coordinate.clone()
        };
        let _ = writeln!(
            out,
            "{}<joint name=\"{jname}\" type=\"{jtype}\" axis=\"{}\"/>",
            ind(cd),
            v3(ax.axis)
        );
        driven.push(DrivenJoint {
            joint: jname,
            coordinate: ax.coordinate.clone(),
            function: ax.function.clone(),
        });
    }

    // A body that carries a free DOF needs real inertia; a welded body (root /
    // hip-at-neutral) needs none (MuJoCo only requires inertia on moving bodies).
    if !free.is_empty() {
        let _ = writeln!(out, "{}{MAIN_INERTIAL}", ind(cd));
    }

    if let Some(body_sites) = ctx.sites.get(b.name.as_str()) {
        for s in body_sites {
            let _ = writeln!(
                out,
                "{}<site name=\"{}\" pos=\"{}\"/>",
                ind(cd),
                s.name,
                v3(s.location)
            );
        }
    }

    if let Some(body_patellae) = ctx.patellae.get(b.name.as_str()) {
        for pat in body_patellae {
            emit_patella(out, cd, pat, driven);
        }
    }

    // Anatomical children.
    for (child_idx, child) in ctx.model.bodies.iter().enumerate() {
        if child.parent == Some(idx) {
            emit_body(ctx, child_idx, cd, out, driven);
        }
    }

    // Close the real body, then the wrappers (inner → outer).
    let _ = writeln!(out, "{}</body>", ind(d));
    while d > depth {
        d -= 1;
        let _ = writeln!(out, "{}</body>", ind(d));
    }
}

/// A patella body: three coupled slide DOFs tracing the moving point's location
/// splines, with the muscle's site at the body origin.
fn emit_patella(out: &mut String, depth: usize, pat: &PatellaSpec, driven: &mut Vec<DrivenJoint>) {
    let _ = writeln!(
        out,
        "{}<body name=\"{}_pat\" pos=\"0 0 0\">",
        ind(depth),
        pat.site
    );
    let c = depth + 1;
    for (suffix, axis, spline) in [
        ("x", "1 0 0", &pat.x),
        ("y", "0 1 0", &pat.y),
        ("z", "0 0 1", &pat.z),
    ] {
        let jname = format!("{}_{suffix}", pat.site);
        let _ = writeln!(
            out,
            "{}<joint name=\"{jname}\" type=\"slide\" axis=\"{axis}\"/>",
            ind(c)
        );
        driven.push(DrivenJoint {
            joint: jname,
            coordinate: pat.coordinate.clone(),
            function: TransformFn::Spline(spline.clone()),
        });
    }
    let _ = writeln!(out, "{}{TINY}", ind(c));
    let _ = writeln!(out, "{}<site name=\"{}\" pos=\"0 0 0\"/>", ind(c), pat.site);
    let _ = writeln!(out, "{}</body>", ind(depth));
}

fn ind(depth: usize) -> String {
    "  ".repeat(depth)
}

fn v3(v: Vector3<f64>) -> String {
    format!("{} {} {}", v.x, v.y, v.z)
}

/// The basis-axis letter of a unit axis, for naming slide joints. Falls back to
/// the dominant component (A1 knee translations are exact basis axes).
fn axis_letter(axis: Vector3<f64>) -> char {
    let a = [axis.x.abs(), axis.y.abs(), axis.z.abs()];
    let i = (0..3).max_by(|&p, &q| a[p].total_cmp(&a[q])).unwrap_or(0);
    ['x', 'y', 'z'][i]
}
