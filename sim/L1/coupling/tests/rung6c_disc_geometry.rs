//! Rung 6c of the geometry-fidelity ladder — **the real intervertebral disc bonded
//! between two rigid vertebrae**.
//!
//! Rung 6b proved the two-way soft↔rigid bond ([`BondedSandwich`]) on a *primitive*
//! block disc: forward conservation (Newton's third law across the coupled interface),
//! two-way tension under flexion, and a compressed disc springing the endplates apart.
//! This rung swaps the block for the **real L4–L5 disc geometry** (`BodyParts3D`
//! `FMA16036`, tet-meshed from its own signed-distance field) and re-proves those
//! forward claims on the actual anatomy — the disc analogue of the rung-2→rung-3 move
//! from a primitive to a real vertebra.
//!
//! ## What it is (and is NOT)
//!
//! A single **homogeneous** Neo-Hookean disc (one solid volume from `FMA16036`, which
//! sits in the L4–L5 gap). The two-material anulus+nucleus split is a later follow-on:
//! `FMA16037` — the ontology's putative "nucleus" — lies a full vertebra lower
//! (z ≈ 918 vs the disc's ≈ 951), so it is *not* this disc's nucleus and is not used.
//! Forward only; differentiability across the bond is rung 6d.
//!
//! ## Measured, nothing asserted via a proxy (the ladder's recurring lesson)
//!
//! - The **anatomical frame is derived from the disc's own geometry**, no axis taken
//!   on faith: the superior–inferior (SI) axis is the disc's *thinnest* principal
//!   extent, the medio-lateral (ML) axis its *widest* — then asserted to align with
//!   the native `z` / `x`, so a mis-oriented mesh is a loud signal, not a silent wrong
//!   axis (rung-4b discipline).
//! - The **endplate faces are derived from the SI extent** (bands at the superior /
//!   inferior surface), not a hardcoded `z = const` plane — the real disc's endplates
//!   are not planar. Each face is a cap at one SI surface: its measured SI spread is
//!   reported and asserted to stay below half the disc thickness (a surface cap, not
//!   the whole disc), and the two cap centroids sit ≈ one disc thickness apart.
//! - **Conservation** is measured from the solver's own assembly (`ΣF`, `ΣM` about a
//!   common point ≈ 0), **two-way tension** from the per-node SI reactions carrying
//!   both signs, and the **restoring moment** from the reduced wrench — the same
//!   oracles as rung 6b, now on the real mesh.
//!
//! Env-gated + license-clean like the other rungs: `#[ignore]` + `$CF_DISC_STL`
//! (`BodyParts3D` `FMA16036` is CC BY-SA, not committed). Run with:
//!
//! ```text
//! CF_DISC_STL=/path/to/FMA16036.stl \
//!   cargo test -p sim-coupling --release \
//!   --test rung6c_disc_geometry -- --ignored --nocapture
//! ```

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use cf_geometry::{Aabb, IndexedMesh};
use mesh_io::load_stl;
use mesh_repair::{RepairParams, repair_mesh};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use nalgebra::{Point3, UnitQuaternion};
use sim_coupling::BondedSandwich;
use sim_mjcf::load_model;
use sim_soft::{
    Aabb3, Mesh, MeshingHints, SdfMeshedTetMesh, Vec3, VertexId, pick_vertices_by_predicate,
};

/// The exact mesh-derived metric oracle (signed distance to a surface): a parry BVH
/// distance composed with a pseudo-normal inside/outside sign (metric by construction).
/// Mirrors `cf-design-tests`'s ladder helper — sim-coupling has no shared `common/`.
type MeshOracle = Signed<TriMeshDistance, PseudoNormalSign>;

/// Load + weld-repair a mesh named by the environment variable `path_var`.
fn load_native(path_var: &str) -> IndexedMesh {
    let path = std::env::var(path_var).unwrap_or_else(|_| panic!("set ${path_var} to a disc STL"));
    let mut mesh = load_stl(&path).unwrap_or_else(|e| panic!("load {path_var}: {e}"));
    let rep = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!(
        "[{path_var}] welded -> {} verts / {} faces ({} welded)",
        mesh.vertices.len(),
        mesh.faces.len(),
        rep.vertices_welded
    );
    mesh
}

/// Build the exact signed distance oracle for a mesh (parry BVH + pseudo-normal sign).
fn oracle(mesh: &IndexedMesh) -> MeshOracle {
    let dist = TriMeshDistance::new(mesh.clone()).unwrap();
    let sign = PseudoNormalSign::from_distance(&dist);
    Signed {
        distance: dist,
        sign,
    }
}

// ── Meshing / scene constants (SI metres). The disc is recentred to the origin and
// scaled mm→m at load (rung-3 discipline): the soft solver's convergence tolerance is
// an absolute residual floor, so the native ~950 mm coordinates + mm-scale elements
// push the internal forces (`∝ μ·L`) past `tol=1e-10`. A similarity transform to metres
// puts the solve in the same regime as rung 6b; every sign / conservation / thickness
// claim below is scale-invariant. ──
const SCALE: f64 = 1.0e-3; // mm → m
const CELL: f64 = 0.003; // tet lattice spacing (m) — coarse enough for a fast quasi-static solve
const PAD: f64 = 0.0015; // lattice padding beyond the disc AABB (m)
const BAND_FRAC: f64 = 0.18; // endplate band = 18% of the disc's SI extent
const H_BOX: f64 = 0.006; // rigid-vertebra box half-thickness (m)
const STATIC_DT: f64 = 1.0e3; // quasi-static disc (inertial term negligible)

const LOWER: usize = 1; // body index of the inferior vertebra (world = 0)
const UPPER: usize = 2; // body index of the superior vertebra

/// The disc's principal-extent frame, derived from its AABB (no axis on faith).
struct DiscFrame {
    si: usize, // superior–inferior axis index (thinnest extent)
    ml: usize, // medio-lateral axis index (widest extent)
    si_lo: f64,
    si_hi: f64, // surface SI extent along `si`
}

impl DiscFrame {
    fn si_hat(&self) -> Vec3 {
        let mut v = Vec3::zeros();
        v[self.si] = 1.0;
        v
    }
}

/// Recentre the disc to the origin and scale mm→m in place (see the constants block):
/// a similarity transform that puts the solve in the soft solver's SI regime without
/// altering the geometry or any scale-invariant measurement.
fn recenter_and_scale(mesh: &mut IndexedMesh) {
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let center = bbox.min.coords + (bbox.max - bbox.min) * 0.5;
    for v in &mut mesh.vertices {
        *v = Point3::from((v.coords - center) * SCALE);
    }
}

/// Derive the SI (thinnest) and ML (widest) axes from the disc surface AABB, and
/// assert they align with the native `BodyParts3D` `z` (SI) / `x` (ML) — a mis-oriented
/// mesh trips the assert rather than silently rotating the load.
fn disc_frame(bbox: &Aabb) -> DiscFrame {
    let span = bbox.max - bbox.min;
    let spans = [span.x, span.y, span.z];
    let si = (0..3)
        .min_by(|&a, &b| spans[a].total_cmp(&spans[b]))
        .unwrap();
    let ml = (0..3)
        .max_by(|&a, &b| spans[a].total_cmp(&spans[b]))
        .unwrap();
    println!(
        "disc spans = ({:.4},{:.4},{:.4}) m  ->  SI=axis {si} (thinnest), ML=axis {ml} (widest)",
        span.x, span.y, span.z
    );
    assert_eq!(si, 2, "expected SI = native z for BodyParts3D coords");
    assert_eq!(ml, 0, "expected ML = native x for BodyParts3D coords");
    DiscFrame {
        si,
        ml,
        si_lo: bbox.min[si],
        si_hi: bbox.max[si],
    }
}

/// Mean rest position of a vertex set (a bonded face's centroid).
fn centroid(tet: &SdfMeshedTetMesh, verts: &[VertexId]) -> Vec3 {
    let sum: Vec3 = verts.iter().map(|&v| tet.positions()[v as usize]).sum();
    sum / verts.len() as f64
}

/// `(lo, hi)` SI-coordinate spread of a vertex set.
fn si_spread(tet: &SdfMeshedTetMesh, verts: &[VertexId], si: usize) -> (f64, f64) {
    verts
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
            let s = tet.positions()[v as usize][si];
            (lo.min(s), hi.max(s))
        })
}

/// Two free-joint boxes centred on the disc's inferior / superior endplate centroids,
/// each offset by `H_BOX` along SI so the box face meets the disc surface. The boxes
/// are the rigid vertebrae the disc bonds to; gravity off (probe-driven readout).
fn fsu_mjcf(frame: &DiscFrame, c_inf: Vec3, c_sup: Vec3) -> String {
    let si = frame.si_hat();
    let lo = c_inf - si * H_BOX; // inferior box COM (top face at the inferior endplate)
    let hi = c_sup + si * H_BOX; // superior box COM (bottom face at the superior endplate)
    // Box half-sizes (metres): broad in-plane (cover the endplate), thin along SI. Only
    // the body pose feeds the bond; the geometry is cosmetic (probe never steps the
    // rigid engine), but sized realistically for a vertebra endplate.
    let (mut sz, si_i) = ([0.025_f64, 0.025, 0.025], frame.si);
    sz[si_i] = H_BOX;
    format!(
        r#"<mujoco>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="inferior" pos="{lx} {ly} {lz}">
      <freejoint/>
      <geom type="box" size="{sx} {sy} {szz}" mass="0.05"/>
    </body>
    <body name="superior" pos="{ux} {uy} {uz}">
      <freejoint/>
      <geom type="box" size="{sx} {sy} {szz}" mass="0.05"/>
    </body>
  </worldbody>
</mujoco>"#,
        lx = lo.x,
        ly = lo.y,
        lz = lo.z,
        ux = hi.x,
        uy = hi.y,
        uz = hi.z,
        sx = sz[0],
        sy = sz[1],
        szz = sz[2],
    )
}

/// The assembled rung-6c scene + the geometry references the measurements need.
struct Scene {
    sandwich: BondedSandwich<SdfMeshedTetMesh>,
    frame: DiscFrame,
    /// Superior-body COM at rest (the compression/flexion reference).
    rest_upper_com: Vec3,
    /// Inferior / superior endplate centroids (rest), for the placement + pivot.
    c_inf: Vec3,
    c_sup: Vec3,
}

/// The full rung-6c scene: load + tet-mesh the real disc, derive its frame + endplate
/// faces, pose two rigid vertebrae to its surfaces, and bond.
fn build() -> Scene {
    let mut mesh = load_native("CF_DISC_STL");
    recenter_and_scale(&mut mesh);
    let sdf = oracle(&mesh);
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let frame = disc_frame(&bbox);

    // Tet-mesh the disc from its own signed field over a lightly padded AABB.
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(bbox.min.x - PAD, bbox.min.y - PAD, bbox.min.z - PAD),
            Vec3::new(bbox.max.x + PAD, bbox.max.y + PAD, bbox.max.z + PAD),
        ),
        cell_size: CELL,
        material_field: None,
    };
    let tet = SdfMeshedTetMesh::from_sdf(&sdf, &hints).expect("disc meshes");
    println!(
        "tet disc: {} verts, {} tets, {} boundary faces (cell={CELL} m)",
        tet.n_vertices(),
        tet.n_tets(),
        tet.boundary_faces().len()
    );

    // Endplate faces = bands at the SI surface extremes (field-derived, not z=const).
    let band = BAND_FRAC * (frame.si_hi - frame.si_lo);
    let si = frame.si;
    let inferior = pick_vertices_by_predicate(&tet, |p| p[si] < frame.si_lo + band);
    let superior = pick_vertices_by_predicate(&tet, |p| p[si] > frame.si_hi - band);
    assert!(
        !inferior.is_empty() && !superior.is_empty(),
        "endplate bands must be non-empty"
    );
    let (inf_lo, inf_hi) = si_spread(&tet, &inferior, si);
    let (sup_lo, sup_hi) = si_spread(&tet, &superior, si);
    let disc_thickness = frame.si_hi - frame.si_lo;
    assert!(
        (inf_hi - inf_lo) < 0.5 * disc_thickness && (sup_hi - sup_lo) < 0.5 * disc_thickness,
        "each endplate cap should be a surface band, not span half the disc thickness"
    );
    println!(
        "endplates (band {:.1} mm): inferior {} verts SI∈[{:.1},{:.1}] mm, superior {} verts SI∈[{:.1},{:.1}] mm",
        band * 1e3,
        inferior.len(),
        inf_lo * 1e3,
        inf_hi * 1e3,
        superior.len(),
        sup_lo * 1e3,
        sup_hi * 1e3,
    );

    let c_inf = centroid(&tet, &inferior);
    let c_sup = centroid(&tet, &superior);

    let model = load_model(&fsu_mjcf(&frame, c_inf, c_sup)).expect("FSU MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    let rest_upper_com = data.xpos[UPPER];

    let sandwich = BondedSandwich::from_tet_mesh(
        model, data, LOWER, UPPER, tet, inferior, superior, STATIC_DT,
    );
    Scene {
        sandwich,
        frame,
        rest_upper_com,
        c_inf,
        c_sup,
    }
}

/// Total reaction wrench `(ΣF, ΣM about `p0`)` over BOTH bonded faces, from the
/// sandwich's last per-DOF reaction + the world targets it was read at. For a
/// self-equilibrated internal-force field this is `(0, 0)` up to the free-node
/// residual — the conservation oracle (SUT == the solver's own assembly).
fn total_reaction_wrench(c: &BondedSandwich<SdfMeshedTetMesh>, p0: Vec3) -> (Vec3, Vec3) {
    let react = c.last_reaction();
    let targets = c.last_targets();
    let (mut force, mut moment) = (Vec3::zeros(), Vec3::zeros());
    for &v in c.lower_face().iter().chain(c.upper_face()) {
        let i = v as usize;
        let f = Vec3::new(react[3 * i], react[3 * i + 1], react[3 * i + 2]);
        let r = Vec3::new(targets[3 * i], targets[3 * i + 1], targets[3 * i + 2]);
        force += f;
        moment += (r - p0).cross(&f);
    }
    (force, moment)
}

/// `(min, max)` of the per-node SI reaction over the superior bonded face — the
/// two-way-tension probe (a unilateral bond would give `min ≥ 0`).
fn superior_face_si_reaction_range(c: &BondedSandwich<SdfMeshedTetMesh>, si: usize) -> (f64, f64) {
    let react = c.last_reaction();
    c.upper_face()
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
            let r = react[3 * v as usize + si];
            (lo.min(r), hi.max(r))
        })
}

#[test]
#[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
fn real_disc_bonds_conserves_and_springs_apart_under_compression() {
    let Scene {
        sandwich,
        frame,
        rest_upper_com: rest_up,
        c_inf,
        c_sup,
    } = build();
    let mut c = sandwich;
    let si = frame.si;

    // Anatomical placement: the bonded faces sit at the disc's real SI surfaces, each a
    // thin cap, ≈ one disc thickness apart.
    let thickness = (c_sup[si] - c_inf[si]).abs();
    println!(
        "endplate centroids: inferior SI={:.1} mm, superior SI={:.1} mm, apart={:.1} mm (disc SI extent {:.1} mm)",
        c_inf[si] * 1e3,
        c_sup[si] * 1e3,
        thickness * 1e3,
        (frame.si_hi - frame.si_lo) * 1e3
    );
    assert!(
        thickness > 0.5 * (frame.si_hi - frame.si_lo),
        "endplates should sit at opposite SI surfaces, not collapsed together"
    );

    // Press the superior vertebra straight down along SI by a small δ, no rotation
    // (a small strain keeps the real disc's boundary tets well inside their SPD region;
    // the reaction sign / conservation claims hold at any nonzero displacement).
    let delta = 1.0e-4; // m (~0.1 mm, ~0.5% axial strain)
    c.set_body_pose(
        UPPER,
        rest_up - frame.si_hat() * delta,
        UnitQuaternion::identity(),
    );
    let comp = c.probe();

    let (f_tot, m_tot) = total_reaction_wrench(&c, Vec3::zeros());
    let scale = comp.force_lower.norm().max(comp.force_upper.norm());
    println!(
        "[COMPRESSION] F_sup.SI={:+.4} F_inf.SI={:+.4} |ΣF|/s={:.2e} |ΣM|/(s·SI)={:.2e}",
        comp.force_upper[si],
        comp.force_lower[si],
        f_tot.norm() / scale,
        m_tot.norm() / (scale * (frame.si_hi - frame.si_lo))
    );
    // (1) Conservation across the coupled interface.
    assert!(
        f_tot.norm() / scale < 1e-9,
        "compression: force not conserved"
    );
    assert!(
        m_tot.norm() / (scale * (frame.si_hi - frame.si_lo)) < 1e-9,
        "compression: moment not conserved"
    );
    // (2) The disc RESISTS compression: pushes the superior plate up (+SI) and the
    //     inferior plate down (−SI) — springs the endplates apart.
    assert!(
        comp.force_upper[si] > 0.0,
        "compressed disc must push the superior plate up"
    );
    assert!(
        comp.force_lower[si] < 0.0,
        "compressed disc must push the inferior plate down"
    );
    // (3) Compression DOMINATES the face: unlike the flat 6b block, a real curved
    //     endplate under axial load bulges laterally, so a tiny tension sliver at the
    //     rim is expected — but it is a negligible fraction of the peak compression
    //     (contrast flexion below, where tension is COMPARABLE to compression). This is
    //     the measured one-way-vs-two-way distinction on the real geometry.
    let (min_si, max_si) = superior_face_si_reaction_range(&c, si);
    println!(
        "[COMPRESSION] superior face rSI∈[{min_si:+.4},{max_si:+.4}], tension/peak={:.1}%",
        100.0 * (-min_si).max(0.0) / max_si
    );
    assert!(max_si > 0.0, "compressed face must carry compression (+SI)");
    assert!(
        min_si > -0.05 * max_si,
        "under axial compression the face should be essentially all compression; \
         tension sliver {min_si:.4} exceeds 5% of peak compression {max_si:.4}"
    );
}

#[test]
#[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
fn real_disc_carries_two_way_tension_under_flexion() {
    let Scene {
        sandwich,
        frame,
        rest_upper_com: rest_up,
        c_sup: pivot,
        ..
    } = build();
    let mut c = sandwich;
    let (si, ml) = (frame.si, frame.ml);

    // Flex the superior vertebra about the ML axis through the superior endplate
    // centroid — one side of the bonded face lifts (tension), the other presses
    // (compression).
    let theta = 0.015_f64; // ~0.86° (small: keeps the boundary tets inside their SPD region)
    let mut ml_axis = Vec3::zeros();
    ml_axis[ml] = 1.0;
    let rot = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(ml_axis), theta);
    let flexed = pivot + rot * (rest_up - pivot);
    c.set_body_pose(UPPER, flexed, rot);
    let flex = c.probe();

    let (f_tot, m_tot) = total_reaction_wrench(&c, Vec3::zeros());
    let scale = flex.force_lower.norm().max(flex.force_upper.norm());
    let (min_si, max_si) = superior_face_si_reaction_range(&c, si);
    println!(
        "[FLEXION] face rSI∈[{min_si:+.4},{max_si:+.4}] M_sup.ML={:+.5} |ΣF|/s={:.2e} |ΣM|/(s·SI)={:.2e}",
        flex.moment_upper[ml],
        f_tot.norm() / scale,
        m_tot.norm() / (scale * (frame.si_hi - frame.si_lo))
    );
    // (1) Conservation still holds on the real mesh.
    assert!(f_tot.norm() / scale < 1e-9, "flexion: force not conserved");
    assert!(
        m_tot.norm() / (scale * (frame.si_hi - frame.si_lo)) < 1e-9,
        "flexion: moment not conserved"
    );
    // (2) TWO-WAY: the bonded face carries BOTH tension and compression, comparable in
    //     magnitude (not a marginal numerical sliver) — the property a unilateral
    //     penalty plane cannot have.
    assert!(
        min_si < 0.0 && max_si > 0.0,
        "flexed face must carry both tension (min<0) and compression (max>0): [{min_si:.4},{max_si:.4}]"
    );
    assert!(
        min_si.abs() > 0.2 * max_si,
        "tension {min_si:.4} should be comparable to compression {max_si:.4}, not a sliver"
    );
    // (3) RESTORING: a +ML tilt is opposed by a −ML moment on the superior plate.
    assert!(
        flex.moment_upper[ml] < 0.0,
        "flexed disc must produce a restoring moment (M_sup.ML<0), got {:+.5}",
        flex.moment_upper[ml]
    );
}
