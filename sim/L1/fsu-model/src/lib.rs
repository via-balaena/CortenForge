//! The assembled **Functional Spinal Unit** (FSU) flexion model.
//!
//! `cf-fsu-geometry` turns a raw vertebra/disc mesh into static anatomical geometry;
//! this crate turns it into a *live, simulatable* FSU. Two layers:
//!
//! - the **bonded soft disc** ([`build_bonded_disc`] / [`BondedDisc`]): tet-meshes the
//!   real intervertebral disc from its own signed field and bonds it between two rigid
//!   vertebra-endplate boxes ([`BondedSandwich`]), then drives its quasi-static
//!   flexion/extension response;
//! - the **coupled FSU** ([`CoupledFsu`]): assembles the disc (as a
//!   linearised bushing), the ligaments (tendons), and the facets (oriented SDF contact)
//!   into ONE model and solves for the equilibrium pose under an applied moment — the
//!   force-driven, ROM-limited segment, vs rung 7's analytic superposition of the parts.
//!
//! ## Two frames, one bridge
//!
//! The disc solves in its own recentred **SI-metre** frame — the soft solver's
//! convergence tolerance is an absolute residual floor, so the native ~950 mm
//! coordinates must be recentred to the origin and scaled mm→m first (rung-6c
//! discipline). The vertebrae / ligaments / facets, by contrast, live in native
//! millimetres. The map is a pure translate + uniform scale, so
//! [`BondedDisc::center_native`] (the pivot) and the scale bridge the two: a solved
//! node at `p_si` sits at `center_native + p_si / scale` in native mm, and the
//! flexion axis ([`BondedDisc::ml_axis`]) is the same coordinate direction in both.
//!
//! ## Scope / honesty
//!
//! The bonded disc only converges at **sub-degree** strains — beyond ~1° the boundary
//! tets leave their SPD region and the soft solve diverges (and panics) — so
//! [`BondedDisc::flexion_moment`] is a small-angle probe, and a segment's larger-angle
//! range of motion is a linear extrapolation of it (see rung 7).
//!
//! Note the deliberate API asymmetry: [`build_bonded_disc`] returns [`Result`], but the
//! drive methods **panic** on non-convergence rather than returning one — they inherit
//! the fail-close contract of [`BondedSandwich::probe`](sim_coupling::BondedSandwich),
//! which aborts loudly on a diverged solve. Consumers keep `|theta|` inside the
//! validated sub-degree range (a fixed safe sweep never diverges). Threading a
//! recoverable `Result` through the probe is a sim-coupling change deferred until a
//! consumer must drive to genuinely unknown angles.

use anyhow::{Context, Result, bail};
use cf_fsu_geometry::oracle;
use cf_geometry::{Aabb, IndexedMesh};
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_coupling::BondedSandwich;
use sim_mjcf::load_model;
use sim_soft::{
    Aabb3, MaterialField, Mesh, MeshingHints, SdfMeshedTetMesh, Vec3, pick_vertices_by_predicate,
    referenced_vertices,
};
// Re-exported: `FlexionTrajectory::boundary_faces` is `Vec<[VertexId; 3]>`, so consumers
// (e.g. a viewer building a mesh from it) need to name the vertex-index type.
pub use sim_soft::VertexId;

mod coupled;
pub use coupled::{CoupledFrame, CoupledFsu, CoupledParams, CoupledTrajectory};

/// Body index of the inferior (lower) vertebra box in the disc scene (world = 0).
const LOWER: usize = 1;
/// Body index of the superior (upper) vertebra box.
const UPPER: usize = 2;

/// Tunable parameters for [`build_bonded_disc`]. [`Default`] reproduces the
/// rung-6c/7 disc recipe exactly.
#[derive(Debug, Clone, Copy)]
pub struct DiscParams {
    /// Neo-Hookean shear modulus μ (Pa); the first Lamé parameter is `λ = 4μ`.
    pub mu: f64,
    /// Similarity scale applied at load (native mm → solver metres). The soft
    /// solver's tolerance is an absolute residual floor, so the disc is recentred +
    /// scaled into SI metres before meshing (rung-6c discipline).
    pub scale: f64,
    /// Tet lattice spacing in the solver frame (m).
    pub cell: f64,
    /// Lattice padding beyond the disc AABB (m).
    pub pad: f64,
    /// Endplate band thickness as a fraction of the disc's SI extent.
    pub band_frac: f64,
    /// Rigid vertebra-box half-thickness (m).
    pub h_box: f64,
    /// Quasi-static timestep (large, so the inertial term `M/dt²` is negligible).
    pub static_dt: f64,
}

impl Default for DiscParams {
    fn default() -> Self {
        Self {
            mu: 1.0e5,
            scale: 1.0e-3,
            cell: 0.003,
            pad: 0.0015,
            band_frac: 0.18,
            h_box: 0.006,
            static_dt: 1.0e3,
        }
    }
}

/// A live bonded intervertebral disc.
///
/// The real disc geometry tet-meshed and bonded between two rigid vertebra-endplate
/// boxes, plus the frame data a flexion probe needs. Build it with [`build_bonded_disc`].
pub struct BondedDisc {
    sandwich: BondedSandwich<SdfMeshedTetMesh>,
    /// The medio-lateral (flexion/extension) axis: a coordinate unit vector.
    ml_axis: Vector3<f64>,
    /// The superior box's body-origin position at rest, in the solver SI frame.
    rest_upper: Vec3,
    /// The disc AABB centre in native mm — the shared flexion pivot.
    center_native: Point3<f64>,
    /// The native-mm → solver-metre similarity scale (`params.scale`), retained so
    /// [`Self::deformed_nodes_native`] can invert the solve frame back to native mm
    /// (`center_native + p_si / scale`). Rendering is its first consumer.
    scale: f64,
}

/// The two-box disc scene: free-joint inferior / superior vertebra boxes whose COMs
/// sit `h_box` beyond each endplate centroid along SI, so each box face meets a disc
/// surface. Gravity off — the scene is probe-driven.
fn disc_mjcf(c_inf: Vec3, c_sup: Vec3, h_box: f64) -> String {
    let lo = c_inf - Vec3::z() * h_box;
    let hi = c_sup + Vec3::z() * h_box;
    format!(
        r#"<mujoco><option gravity="0 0 0" timestep="0.001"/><worldbody>
    <body name="inf" pos="{lx} {ly} {lz}"><freejoint/><geom type="box" size="0.025 0.025 {h}" mass="0.05"/></body>
    <body name="sup" pos="{ux} {uy} {uz}"><freejoint/><geom type="box" size="0.025 0.025 {h}" mass="0.05"/></body>
    </worldbody></mujoco>"#,
        lx = lo.x,
        ly = lo.y,
        lz = lo.z,
        ux = hi.x,
        uy = hi.y,
        uz = hi.z,
        h = h_box,
    )
}

/// Tet-mesh the real intervertebral disc `mesh` (native mm) and bond it between two
/// field-posed rigid endplate boxes, returning the live [`BondedDisc`].
///
/// The disc is recentred to its AABB centre and scaled by `params.scale` into the
/// solver's SI-metre frame; its principal axes are derived from the AABB (SI =
/// thinnest, ML/flexion = widest — no axis taken on faith).
///
/// # Errors
/// Returns an error if the disc's thinnest extent is not its native `z` (a
/// mis-oriented mesh), if an endplate band captures no vertices (band/cell too coarse
/// for the mesh), if the two endplate bands overlap (`band_frac` too large — they
/// would share a vertex), or if the signed-distance oracle, tet-mesher, or MJCF scene
/// build fails.
// `vs.len()` (an endplate vertex count) is tiny; the usize→f64 cast for its centroid
// is exact for any real mesh.
#[allow(clippy::cast_precision_loss)]
pub fn build_bonded_disc(mut mesh: IndexedMesh, params: &DiscParams) -> Result<BondedDisc> {
    let bbox0 = Aabb::from_points(mesh.vertices.iter());
    let center_native = Point3::from(bbox0.min.coords + (bbox0.max - bbox0.min) * 0.5);
    for v in &mut mesh.vertices {
        *v = Point3::from((v.coords - center_native.coords) * params.scale);
    }
    let bbox = Aabb::from_points(mesh.vertices.iter());

    // Principal axes from the AABB (rung-6c discipline): SI = thinnest (guarded to be
    // native z), ML = widest (the flexion/extension axis) — no axis taken on faith.
    let size = bbox.size();
    if !(size.z < size.x && size.z < size.y) {
        bail!(
            "disc SI extent (thinnest) must be native z; got extents ({:.4}, {:.4}, {:.4}) m — mis-oriented mesh?",
            size.x,
            size.y,
            size.z
        );
    }
    // ML = the widest extent. Deliberately NOT `Aabb::longest_axis()`: that breaks an
    // exact-tie between extents as first-max (X), whereas rung-7's original `max_by`
    // returned the LAST maximum — matching it keeps this a byte-identical extraction. (A
    // real anatomical disc has a unique widest axis, so the two agree on every real
    // input; they differ only for a physically degenerate exactly-square disc.)
    let extents = [size.x, size.y, size.z];
    let widest = (0..3)
        .max_by(|&a, &b| extents[a].total_cmp(&extents[b]))
        .unwrap_or(0);
    let mut ml_axis = Vector3::zeros();
    ml_axis[widest] = 1.0;

    // Pad the lattice beyond the disc so the tet mesh fully contains the surface.
    let padded = bbox.expanded(params.pad);
    let hints = MeshingHints {
        bbox: Aabb3::new(
            Vec3::new(padded.min.x, padded.min.y, padded.min.z),
            Vec3::new(padded.max.x, padded.max.y, padded.max.z),
        ),
        cell_size: params.cell,
        material_field: Some(MaterialField::uniform(params.mu, 4.0 * params.mu)),
    };
    // Build the exact SDF only once the cheap bbox guards have passed (a mis-oriented
    // mesh bails above without paying for the oracle).
    let sdf = oracle(&mesh).context("disc oracle")?;
    let tet = SdfMeshedTetMesh::from_sdf(&sdf, &hints)
        .map_err(|e| anyhow::anyhow!("tet-mesh disc: {e:?}"))?;
    // A physical disc is one connected solid, but the BCC isosurface-stuffing mesher
    // fragments the disc's sub-cell-thin tapering rim into disconnected islands — which
    // both scatter the rendered surface and poison the Newton tangent's conditioning
    // (a floating tet component carries unconstrained rigid modes). Keep the main body.
    let tet = tet.largest_component();

    // Endplate faces = bands at the SI surface extremes (field-derived, not z=const).
    let (lo_z, hi_z) = (bbox.min.z, bbox.max.z);
    let band = params.band_frac * (hi_z - lo_z);
    // `largest_component` (and the mesher's own lattice) retain unreferenced "orphan"
    // vertices; a spatial predicate over ALL positions can pick them, and bonding a
    // zero-stiffness orphan (or averaging it into the endplate centroid) would silently
    // skew the disc. Drop orphans first — the established `referenced_vertices` pattern.
    let referenced: std::collections::HashSet<VertexId> =
        referenced_vertices(&tet).into_iter().collect();
    let inferior: Vec<VertexId> = pick_vertices_by_predicate(&tet, |p| p.z < lo_z + band)
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();
    let superior: Vec<VertexId> = pick_vertices_by_predicate(&tet, |p| p.z > hi_z - band)
        .into_iter()
        .filter(|v| referenced.contains(v))
        .collect();
    if inferior.is_empty() || superior.is_empty() {
        bail!(
            "endplate band ({:.4} m) captured no vertices (inferior {}, superior {}) — increase band_frac or refine cell",
            band,
            inferior.len(),
            superior.len()
        );
    }
    // The two bands must be DISJOINT: a vertex in both would bond one disc node to both
    // vertebrae, which `BondedSandwich::from_tet_mesh` rejects with a panic. A
    // `band_frac ≥ 0.5` makes the bands meet/overlap at the mid-plane. Catch it here as
    // a recoverable error (the `# Errors` contract) rather than a downstream panic.
    let inferior_set: std::collections::HashSet<VertexId> = inferior.iter().copied().collect();
    if superior.iter().any(|v| inferior_set.contains(v)) {
        bail!(
            "inferior/superior endplate bands overlap (band_frac {:.2} too large) — they share a vertex, which cannot bond to both endplates",
            params.band_frac
        );
    }
    let cen = |vs: &[VertexId]| -> Vec3 {
        let s: Vec3 = vs.iter().map(|&v| tet.positions()[v as usize]).sum();
        s / vs.len() as f64
    };
    let (c_inf, c_sup) = (cen(&inferior), cen(&superior));

    let model = load_model(&disc_mjcf(c_inf, c_sup, params.h_box)).context("disc scene MJCF")?;
    let mut data = model.make_data();
    data.forward(&model).context("disc scene forward")?;
    let rest_upper = data.xpos[UPPER];
    let sandwich = BondedSandwich::from_tet_mesh(
        model,
        data,
        LOWER,
        UPPER,
        tet,
        inferior,
        superior,
        params.static_dt,
    );
    Ok(BondedDisc {
        sandwich,
        ml_axis,
        rest_upper,
        center_native,
        scale: params.scale,
    })
}

/// One captured flexion pose: the imposed angle, the disc's deformed surface (native
/// mm), and the small-angle restoring response + conservation residual at that pose.
pub struct FlexionFrame {
    /// The imposed flexion angle about the ML axis (rad).
    pub theta: f64,
    /// The disc's deformed tet-vertex positions in **native millimetres**, ready to
    /// pair with [`FlexionTrajectory::boundary_faces`] for the deformed surface.
    pub deformed_nodes_native: Vec<Point3<f64>>,
    /// The superior-endplate restoring moment about the disc centre, projected on the
    /// ML axis (N·m) — negative for a restoring response (see [`BondedDisc::flexion_moment`]).
    pub moment: f64,
    /// The bond's conservation residual `‖ΣF‖ + ‖ΣM‖` over both endplates (≈ 0).
    pub conservation_resid: f64,
}

/// A replayable capture of a [`BondedDisc`] flexion sweep.
///
/// Holds the shared pivot + axis, the disc's rest surface, its (deformation-invariant)
/// boundary triangulation, and one [`FlexionFrame`] per swept angle — everything a
/// viewer needs to replay the disc deforming while the superior vertebra rotates about
/// `(pivot, axis, theta)`.
///
/// All positions are **native millimetres** (the vertebra/ligament/facet frame), so a
/// renderer never touches the solver's SI frame. Produced by [`BondedDisc::capture_flexion`].
pub struct FlexionTrajectory {
    /// The flexion pivot — the disc AABB centre in native mm ([`BondedDisc::center_native`]).
    pub pivot: Point3<f64>,
    /// The flexion axis — the disc ML unit vector ([`BondedDisc::ml_axis`]).
    pub axis: Vector3<f64>,
    /// The disc's rest (θ = 0 equilibrium) tet-vertex positions in native mm — the
    /// reference a viewer exaggerates deformation against.
    pub rest_nodes_native: Vec<Point3<f64>>,
    /// The disc surface triangulation, indexing into every frame's
    /// `deformed_nodes_native` (constant across the sweep; see [`BondedDisc::boundary_faces`]).
    pub boundary_faces: Vec<[VertexId; 3]>,
    /// The captured poses, one per swept angle.
    pub frames: Vec<FlexionFrame>,
}

impl BondedDisc {
    /// The medio-lateral (flexion/extension) axis — a coordinate unit vector,
    /// identical in the solver SI frame and native mm (the map is translate + scale).
    #[must_use]
    pub const fn ml_axis(&self) -> Vector3<f64> {
        self.ml_axis
    }

    /// The shared flexion pivot: the disc's AABB centre in **native millimetres**.
    #[must_use]
    pub const fn center_native(&self) -> Point3<f64> {
        self.center_native
    }

    /// The disc's current deformed tet-vertex positions mapped back to **native
    /// millimetres** (`center_native + p_si / scale`), inverting the recentre + scale
    /// applied at build. Reads the last solved configuration, so call it after
    /// [`Self::set_flexion`] / [`Self::flexion_moment`] to read that pose's surface.
    ///
    /// Pairs with [`Self::boundary_faces`] to build the deformed disc surface in the
    /// same native-mm frame as the vertebrae, ligaments, and facets.
    #[must_use]
    pub fn deformed_nodes_native(&self) -> Vec<Point3<f64>> {
        let x = self.sandwich.soft_positions();
        (0..x.len() / 3)
            .map(|i| {
                let p_si = Vector3::new(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
                self.center_native + p_si / self.scale
            })
            .collect()
    }

    /// The disc surface triangulation (outward-oriented boundary faces), indexing into
    /// [`Self::deformed_nodes_native`]. Constant across a flexion sweep — only the vertex
    /// positions deform — so a viewer snapshots it once and rebuilds each frame's mesh.
    #[must_use]
    pub fn boundary_faces(&self) -> &[[VertexId; 3]] {
        self.sandwich.soft_boundary_faces()
    }

    /// Sweep a sequence of flexion `angles` (rad) and record a replayable
    /// [`FlexionTrajectory`]: the shared pivot/axis, the rest surface, the boundary
    /// triangulation, and one [`FlexionFrame`] per angle (deformed surface + restoring
    /// moment + conservation residual). On return the disc is left at the last swept
    /// angle; the recorded rest surface is independent of that (see below).
    ///
    /// The rest surface is the θ = 0 equilibrium, solved first so it is independent of any
    /// prior drive state. Each frame is a single quasi-static solve (via
    /// [`Self::flexion_moment`]), then the deformed surface is read back in native mm — so
    /// a capture costs exactly `N + 1` solves (rest + one per angle).
    ///
    /// # Panics
    /// Panics if any angle drives the soft solve past its SPD region — see
    /// [`Self::set_flexion`]. Keep every `|angle|` inside the validated sub-degree range.
    #[must_use]
    pub fn capture_flexion(&mut self, angles: &[f64]) -> FlexionTrajectory {
        // Rest = the θ = 0 equilibrium, solved up front so it does not depend on whatever
        // pose a prior caller left the disc in.
        self.set_flexion(0.0);
        let rest_nodes_native = self.deformed_nodes_native();
        let boundary_faces = self.boundary_faces().to_vec();

        let frames = angles
            .iter()
            .map(|&theta| {
                let (moment, conservation_resid) = self.flexion_moment(theta);
                FlexionFrame {
                    theta,
                    deformed_nodes_native: self.deformed_nodes_native(),
                    moment,
                    conservation_resid,
                }
            })
            .collect();

        FlexionTrajectory {
            pivot: self.center_native,
            axis: self.ml_axis,
            rest_nodes_native,
            boundary_faces,
            frames,
        }
    }

    /// Impose flexion angle `theta` (rad) about the ML axis through the disc centre
    /// and re-solve the quasi-static bond: the superior endplate box rotates about the
    /// SI-frame origin (the bonded face follows rigidly), the inferior stays at rest.
    ///
    /// Keep `|theta|` sub-degree — the bond converges only while the boundary tets stay
    /// in their SPD region.
    ///
    /// # Panics
    /// Panics if the quasi-static soft solve fails to converge — a `|theta|` large
    /// enough to drive the boundary tets out of their SPD region (beyond ~1°) will
    /// exceed the Newton iteration cap and abort.
    pub fn set_flexion(&mut self, theta: f64) {
        let rot = UnitQuaternion::from_axis_angle(&Unit::new_normalize(self.ml_axis), theta);
        self.sandwich
            .set_body_pose(UPPER, rot * self.rest_upper, rot);
        self.sandwich.probe();
    }

    /// Impose flexion `theta` (rad) and measure the disc's restoring response: the
    /// reaction moment on the superior endplate about the disc centre projected on the
    /// ML axis (N·m), plus a conservation residual `‖ΣF‖ + ‖ΣM‖` over both bonded faces
    /// (≈ 0 for a self-equilibrated field — the rung-6 oracle).
    ///
    /// # Panics
    /// Panics if the soft solve diverges — see [`Self::set_flexion`]; keep `|theta|`
    /// sub-degree.
    pub fn flexion_moment(&mut self, theta: f64) -> (f64, f64) {
        self.set_flexion(theta);
        let react = self.sandwich.last_reaction();
        let targets = self.sandwich.last_targets();
        let at = |i: usize, s: &[f64]| Vec3::new(s[3 * i], s[3 * i + 1], s[3 * i + 2]);
        // Moment on the superior vertebra about the origin (disc centre).
        let mut m_up = Vec3::zeros();
        for &v in self.sandwich.upper_face() {
            let i = v as usize;
            m_up += at(i, targets).cross(&at(i, react));
        }
        // Conservation over BOTH faces (should vanish).
        let (mut f_tot, mut m_tot) = (Vec3::zeros(), Vec3::zeros());
        for &v in self
            .sandwich
            .lower_face()
            .iter()
            .chain(self.sandwich.upper_face())
        {
            let i = v as usize;
            let f = at(i, react);
            f_tot += f;
            m_tot += at(i, targets).cross(&f);
        }
        (m_up.dot(&self.ml_axis), f_tot.norm() + m_tot.norm())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

    use super::*;

    /// A watertight axis-aligned box (8 verts, 12 outward-wound triangles) with the
    /// given half-extents, centred at `center` — a closed surface the signed oracle
    /// can sign, standing in for the disc (thinnest extent = a disc-like endplate gap).
    ///
    /// Mirrors the cubic `box_mesh` fixture in `cf-fsu-geometry`'s own tests (widened
    /// here to per-axis half-extents). Kept local rather than shared: it is a
    /// `#[cfg(test)]` helper, and promoting it to either crate's public surface just to
    /// avoid a small, self-contained fixture would leak test scaffolding into the API.
    fn box_mesh(center: Point3<f64>, half: Vector3<f64>) -> IndexedMesh {
        let c = center;
        let (hx, hy, hz) = (half.x, half.y, half.z);
        let vertices = vec![
            Point3::new(c.x - hx, c.y - hy, c.z - hz),
            Point3::new(c.x + hx, c.y - hy, c.z - hz),
            Point3::new(c.x + hx, c.y + hy, c.z - hz),
            Point3::new(c.x - hx, c.y + hy, c.z - hz),
            Point3::new(c.x - hx, c.y - hy, c.z + hz),
            Point3::new(c.x + hx, c.y - hy, c.z + hz),
            Point3::new(c.x + hx, c.y + hy, c.z + hz),
            Point3::new(c.x - hx, c.y + hy, c.z + hz),
        ];
        let faces = vec![
            [0, 3, 2],
            [0, 2, 1], // z-
            [4, 5, 6],
            [4, 6, 7], // z+
            [0, 1, 5],
            [0, 5, 4], // y-
            [2, 3, 7],
            [2, 7, 6], // y+
            [0, 4, 7],
            [0, 7, 3], // x-
            [1, 2, 6],
            [1, 6, 5], // x+
        ];
        IndexedMesh { vertices, faces }
    }

    /// A disc-like synthetic slab: widest in x (ML), thinnest in z (SI), placed at a
    /// native-mm-scale offset so recentring + scaling are exercised.
    fn synthetic_disc() -> IndexedMesh {
        box_mesh(
            Point3::new(100.0, 100.0, 950.0),
            Vector3::new(12.0, 10.0, 3.0), // 24 × 20 × 6 mm — x widest, z thinnest
        )
    }

    #[test]
    fn builds_and_derives_the_ml_axis_and_pivot() {
        let disc = build_bonded_disc(synthetic_disc(), &DiscParams::default()).unwrap();
        // ML = widest extent = native x; pivot = the AABB centre in native mm.
        assert_eq!(
            disc.ml_axis(),
            Vector3::x(),
            "widest extent (x) is the ML axis"
        );
        assert!(
            (disc.center_native() - Point3::new(100.0, 100.0, 950.0)).norm() < 1e-9,
            "pivot is the native-mm AABB centre, got {:?}",
            disc.center_native()
        );
    }

    #[test]
    fn flexion_is_restoring_conserving_and_antisymmetric() {
        let mut disc = build_bonded_disc(synthetic_disc(), &DiscParams::default()).unwrap();
        let theta = 0.3_f64.to_radians(); // sub-degree: stay in the SPD region

        let (m_pos, resid_pos) = disc.flexion_moment(theta);
        let (m_neg, resid_neg) = disc.flexion_moment(-theta);

        // Conservation: the bond's reaction is a self-equilibrated wrench.
        assert!(
            resid_pos < 1e-9 && resid_neg < 1e-9,
            "bond must conserve (residuals {resid_pos:.2e}, {resid_neg:.2e})"
        );
        // Restoring: a +θ tilt is opposed by a −ML moment on the superior plate.
        assert!(
            m_pos < 0.0,
            "flexion must be restoring (M·ML < 0), got {m_pos:+.3e}"
        );
        // Small-strain linearity ⇒ the response is antisymmetric in θ.
        let asym = (m_pos + m_neg).abs() / m_pos.abs().max(1e-12);
        assert!(
            asym < 0.1,
            "small-strain response should be antisymmetric (asymmetry {asym:.3})"
        );
    }

    #[test]
    fn rejects_a_misoriented_disc() {
        // Thinnest extent along x (not z) — the SI-axis guard must reject it.
        let bad = box_mesh(Point3::origin(), Vector3::new(3.0, 10.0, 12.0));
        let Err(err) = build_bonded_disc(bad, &DiscParams::default()) else {
            panic!("expected the SI-orientation guard to reject a mis-oriented disc");
        };
        assert!(
            format!("{err}").contains("mis-oriented"),
            "expected the SI-orientation guard to fire, got: {err}"
        );
    }

    /// Assert a captured sweep is physically sound and return its max node displacement
    /// (native mm). Shared by the synthetic and real-disc capture tests so the flexion
    /// invariant lives in ONE place: a valid deformation-invariant boundary surface,
    /// per-frame conservation, a strictly restoring moment off neutral / a vanishing one
    /// at neutral, and a real imposed deformation of at least `min_disp` mm. `min_disp` is
    /// passed per-test, tied to that disc's geometry (`extent·sin θ`) and set comfortably
    /// above the ~0.02 mm interior-node solve-noise floor so it tests imposed deformation,
    /// not noise.
    fn assert_restoring_sweep(traj: &FlexionTrajectory, min_disp: f64) -> f64 {
        let n = traj.rest_nodes_native.len();
        assert!(!traj.boundary_faces.is_empty(), "disc must have a surface");
        assert!(
            traj.boundary_faces
                .iter()
                .flatten()
                .all(|&v| (v as usize) < n),
            "every boundary-face vertex must index into the {n}-vertex node buffer"
        );
        // Scale for the neutral-frame check: the largest loaded restoring moment.
        let moment_scale = traj
            .frames
            .iter()
            .map(|f| f.moment.abs())
            .fold(0.0_f64, f64::max);
        let mut max_disp = 0.0_f64;
        for f in &traj.frames {
            assert_eq!(f.deformed_nodes_native.len(), n, "node count constant");
            assert!(
                f.conservation_resid < 1e-8,
                "θ={:.3}° bond must conserve (resid {:.2e})",
                f.theta.to_degrees(),
                f.conservation_resid
            );
            if f.theta.abs() > 1e-9 {
                // Loaded: the ML moment strictly opposes the tilt (θ·M < 0). A `≤ ε` bound
                // would pass a spurious M = 0, so require a strictly restoring sign.
                assert!(
                    f.theta * f.moment < 0.0,
                    "θ={:.3}° must be restoring (θ·M = {:.2e})",
                    f.theta.to_degrees(),
                    f.theta * f.moment
                );
            } else {
                // Neutral: the response must vanish (checked explicitly — `0·M ≤ ε` is
                // vacuously true and would never test the θ=0 moment).
                assert!(
                    f.moment.abs() < 0.1 * moment_scale,
                    "neutral moment must be ~0, got {:.2e} (scale {moment_scale:.2e})",
                    f.moment
                );
            }
            for (p, r) in f.deformed_nodes_native.iter().zip(&traj.rest_nodes_native) {
                max_disp = max_disp.max((p - r).norm());
            }
        }
        assert!(
            max_disp > min_disp,
            "the sweep must impose a real deformation (≥ {min_disp:.2e} mm, above solve noise); got {max_disp:.2e} mm"
        );
        max_disp
    }

    #[test]
    fn capture_flexion_records_a_deforming_restoring_sweep() {
        // License-free coverage of the capture seam on the synthetic disc: exercises
        // capture_flexion + deformed_nodes_native + boundary_faces (the real-anatomy
        // build+measure gate is the #[ignore]d test below).
        let mut disc = build_bonded_disc(synthetic_disc(), &DiscParams::default()).unwrap();
        let angles: Vec<f64> = [-0.3_f64, 0.0, 0.3]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = disc.capture_flexion(&angles);

        // Shared pivot/axis + the direct accessors agree with the trajectory snapshot.
        assert_eq!(traj.pivot, disc.center_native());
        assert_eq!(traj.axis, disc.ml_axis());
        assert_eq!(disc.boundary_faces(), traj.boundary_faces.as_slice());
        assert_eq!(
            disc.deformed_nodes_native().len(),
            traj.rest_nodes_native.len()
        );

        // The 24×20×6 mm slab tilted 0.3° about its ML(x) axis moves its farthest node
        // (~10 mm off-axis) by ~10·sin(0.3°) ≈ 0.05 mm; require ≥ 0.02 mm (above noise).
        assert_restoring_sweep(&traj, 2e-2);
    }

    #[test]
    #[ignore = "needs $CF_DISC_STL (BodyParts3D FMA16036, CC BY-SA, not committed)"]
    fn captures_a_replayable_flexion_sweep_on_the_real_disc() {
        // Build+measure on the real intervertebral disc: capture a validated sub-degree
        // sweep and assert the trajectory a viewer will replay is physically sound and
        // deterministic — no proxy. Gated + license-clean like the rung tests.
        let disc_mesh = cf_fsu_geometry::load_from_env("CF_DISC_STL").expect("load disc mesh");
        let mut disc = build_bonded_disc(disc_mesh, &DiscParams::default()).expect("build disc");

        // Symmetric, all within rung-7's validated SPD range (|θ| ≤ 0.86°).
        let angles: Vec<f64> = [-0.86_f64, -0.5, 0.0, 0.5, 0.86]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = disc.capture_flexion(&angles);

        // Shared pivot/axis are the disc's own.
        assert_eq!(traj.pivot, disc.center_native());
        assert_eq!(traj.axis, disc.ml_axis());

        assert_eq!(traj.frames.len(), angles.len());

        // Physics (shared with the synthetic test): valid surface, conservation, restoring
        // moment, real deformation. The ±0.86° sweep on the real disc deforms it ~0.4 mm;
        // require ≥ 0.1 mm — well above the ~0.02 mm interior-node solve-noise floor.
        let max_disp = assert_restoring_sweep(&traj, 1e-1);
        // The disc ACTUALLY deforms (sub-mm at sub-degree — the viewer exaggerates it).
        println!(
            "captured {} frames; max node displacement {max_disp:.4} mm",
            traj.frames.len()
        );

        // Reproducibility — asserted on the PHYSICAL OBSERVABLE (the restoring moment),
        // not per-node bit-identity. `build_bonded_disc` now drops the mesher's
        // disconnected rim islands (`SdfMeshedTetMesh::largest_component`), which were the
        // dominant source of the near-singular Newton tangent (floating tet components =
        // unconstrained rigid modes): the faer LU fallback count fell from ~17 to ~4 on
        // the real disc. A small residual remains (near-sliver tets within the main body),
        // so the multi-threaded indefinite LU can still land on slightly different interior
        // configurations run-to-run. The bonded (boundary) nodes are Dirichlet-pinned, so
        // the reaction — hence the moment — is well-determined regardless. We assert the
        // observable a regression would actually track. (Fully eliminating the residual
        // needs a thin-feature-capable mesher — a separate sim-soft rung.)
        //
        // Drift is the absolute moment change normalised by the sweep's peak moment, NOT by
        // each frame's own moment: the near-zero θ=0 frame would make a per-frame ratio blow up.
        let moment_scale = traj
            .frames
            .iter()
            .map(|f| f.moment.abs())
            .fold(0.0_f64, f64::max);
        let again = disc.capture_flexion(&angles);
        let mut max_moment_drift = 0.0_f64;
        for (a, b) in traj.frames.iter().zip(&again.frames) {
            max_moment_drift = max_moment_drift.max((a.moment - b.moment).abs());
        }
        let moment_rel = max_moment_drift / moment_scale.max(1e-12);
        println!("re-capture moment drift: {moment_rel:.2e} rel (scale {moment_scale:.2e})");
        assert!(
            moment_rel < 1e-3,
            "the restoring moment must reproduce across captures (drift {moment_rel:.2e} rel)"
        );
    }

    #[test]
    fn rejects_overlapping_endplate_bands() {
        // band_frac ≥ 0.5 makes the two bands meet at the mid-plane and share vertices;
        // build_bonded_disc must return an error, NOT let the shared-vertex panic escape
        // from BondedSandwich::from_tet_mesh.
        let params = DiscParams {
            band_frac: 0.6,
            ..DiscParams::default()
        };
        let Err(err) = build_bonded_disc(synthetic_disc(), &params) else {
            panic!("expected overlapping endplate bands to be rejected");
        };
        assert!(format!("{err}").contains("overlap"), "got: {err}");
    }
}
