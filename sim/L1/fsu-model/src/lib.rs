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
pub use coupled::{
    CoupledFrame, CoupledFsu, CoupledParams, CoupledTrajectory, PHYSIOLOGIC_MOMENT, RAMP_FRAMES,
    is_engaged, moment_ramp, posed_facet_contacts,
};

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

/// The disc's medio-lateral (flexion/extension) axis: the coordinate axis of the point
/// cloud's widest **AABB extent**.
///
/// Deliberately the axis-aligned AABB extent — NOT a PCA principal direction (an oblique
/// disc's longest point-to-point direction can differ from its widest axis-aligned extent),
/// and NOT [`Aabb::longest_axis`] (which resolves an exact extent tie to the FIRST maximum;
/// rung 7's original `max_by` takes the LAST, and a byte-identical extraction must match it).
/// A real anatomical disc has a unique widest axis, so the choices agree on every real input.
fn ml_axis_from_points(vertices: &[Point3<f64>]) -> Vector3<f64> {
    let size = Aabb::from_points(vertices.iter()).size();
    let extents = [size.x, size.y, size.z];
    let widest = (0..3)
        .max_by(|&a, &b| extents[a].total_cmp(&extents[b]))
        .unwrap_or(0);
    let mut ml = Vector3::zeros();
    ml[widest] = 1.0;
    ml
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
    let ml_axis = ml_axis_from_points(&mesh.vertices);

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

/// License-free geometry fixtures shared by this crate's test modules (`lib.rs` and
/// `coupled.rs`), so the box triangulation lives in exactly one place.
#[cfg(test)]
pub(crate) mod test_support {
    use nalgebra::{Point3, Vector3};

    use crate::IndexedMesh;

    /// A watertight axis-aligned box (8 verts, 12 outward-wound triangles) with the given
    /// half-extents, centred at `center` — a closed surface the signed oracle can sign,
    /// standing in for the disc (thinnest extent = a disc-like endplate gap).
    #[must_use]
    pub fn box_mesh(center: Point3<f64>, half: Vector3<f64>) -> IndexedMesh {
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
    #[must_use]
    pub fn synthetic_disc() -> IndexedMesh {
        box_mesh(
            Point3::new(100.0, 100.0, 950.0),
            Vector3::new(12.0, 10.0, 3.0), // 24 × 20 × 6 mm — x widest, z thinnest
        )
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // tests may unwrap/expect/panic.

    use super::test_support::{box_mesh, synthetic_disc};
    use super::*;

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

    /// Rough endplate face selection: the down-facing lower region (`sign = -1`,
    /// L4 inferior) or up-facing upper region (`sign = +1`, L5 superior).
    fn select_endplate(mesh: &IndexedMesh, sign: f64) -> Vec<usize> {
        let bbox = Aabb::from_points(mesh.vertices.iter());
        let zmid = 0.5 * (bbox.min.z + bbox.max.z);
        let mut ids = Vec::new();
        for i in 0..mesh.faces.len() {
            let Some(tri) = mesh.triangle(i) else {
                continue;
            };
            let centroid = Point3::from((tri.v0.coords + tri.v1.coords + tri.v2.coords) / 3.0);
            let normal = (tri.v1 - tri.v0).cross(&(tri.v2 - tri.v0));
            let norm = normal.norm();
            if norm < f64::EPSILON {
                continue;
            }
            let region = if sign < 0.0 {
                centroid.z < zmid
            } else {
                centroid.z > zmid
            };
            if region && (normal.z / norm) * sign > 0.7 {
                ids.push(i);
            }
        }
        ids
    }

    /// Loft a disc from rough auto-selected L4/L5 endplate patches (the painting
    /// GUI does this cleaner).
    fn lofted_disc(l4: &IndexedMesh, l5: &IndexedMesh) -> IndexedMesh {
        use cf_fsu_geometry::loft::{
            WallCorrespondence, assemble_bushing, extract_patch, finalize_patch, flip_patch,
        };
        let l4_faces = select_endplate(l4, -1.0);
        let l5_faces = select_endplate(l5, 1.0);
        assert!(
            !l4_faces.is_empty() && !l5_faces.is_empty(),
            "empty endplate selection"
        );
        // Prepare each patch (largest component, interior holes sealed → one outer
        // rim) so the loft meets assemble_bushing's single-boundary precondition.
        let top = finalize_patch(&extract_patch(l4, &l4_faces));
        let bottom = finalize_patch(&extract_patch(l5, &l5_faces));
        let top = flip_patch(&top);
        // `finalize_patch` already guarantees one connected component with its
        // interior holes sealed; the assembled disc may still carry a few open
        // wall-seam edges (the arc-length correspondence on these dissimilar
        // auto-selected rims), which the SDF tet-mesher resamples away — so we do
        // NOT require strict watertightness here (measured: 50 open edges / 0
        // non-manifold on the real L4/L5, and it tet-meshes, bonds, and sweeps).
        assemble_bushing(&top, &bottom, 1, WallCorrespondence::ArcLength).mesh
    }

    /// B6 end-to-end: a human-lofted disc **tet-meshes, bonds to a restoring
    /// stiffness, seats on the exact bone for free, and sweeps** — the payoff of
    /// building the disc *from* the endplates. The scanned disc could do none of
    /// these (it over-stretched 2.15× and its conform shifted `k_disc` ~6×). The
    /// bond + sweep succeeding is itself proof the tet mesh is clean — inverted or
    /// fragmented tets diverge (spike-measured: 99% kept, 11.8° min dihedral, 0
    /// inverted).
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn b6_lofted_disc_bonds_seats_and_sweeps() {
        use cf_fsu_geometry::{conform_disc_to_endplates, load_from_env, segment_frame};

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let o4 = oracle(&l4).unwrap();
        let o5 = oracle(&l5).unwrap();
        let frame = segment_frame(&l4, &l5, &o4, &o5).unwrap();
        let disc = lofted_disc(&l4, &l5);
        let params = DiscParams::default();

        // (1) The caps ARE the endplate surfaces: conforming them onto the exact
        // bone is a sub-mm move (the scanned disc needed multi-mm, stiffening moves).
        let conformed = conform_disc_to_endplates(&disc, &o4, &o5, &frame, Some(params.band_frac));
        let max_move = disc
            .vertices
            .iter()
            .zip(&conformed.vertices)
            .map(|(a, b)| (a - b).norm())
            .fold(0.0, f64::max);
        println!("conform: max cap-band move {max_move:.2} mm");
        assert!(
            max_move < 4.0,
            "cap band not seated on the bone ({max_move:.2} mm)"
        );

        // (2) Bonds + probes to a restoring, symmetric, self-equilibrated k_disc,
        // and the exact-bone conform leaves it intact (no ~6× shift).
        let theta = 0.5_f64.to_radians();
        let mut raw_bond = build_bonded_disc(disc, &params).expect("lofted disc bonds");
        let (m_flex, resid_flex) = raw_bond.flexion_moment(theta);
        let (m_ext, resid_ext) = raw_bond.flexion_moment(-theta);
        let (k_flex, k_ext) = (m_flex / theta, m_ext / -theta);
        let k_conformed = build_bonded_disc(conformed, &params)
            .expect("conformed disc bonds")
            .flexion_moment(theta)
            .0
            / theta;
        println!(
            "k_disc: flex {k_flex:.1}, ext {k_ext:.1}, conformed {k_conformed:.1} N·m/rad; resid {resid_flex:.1e} / {resid_ext:.1e}"
        );
        assert!(k_flex.is_finite() && k_ext.is_finite(), "non-finite k_disc");
        assert!(
            k_flex * k_ext > 0.0 && k_flex.abs() > 1.0,
            "not a consistent linear spring ({k_flex:.1} vs {k_ext:.1})"
        );
        assert!(
            resid_flex < 1e-2 && resid_ext < 1e-2,
            "bond not self-equilibrated ({resid_flex:.2e} / {resid_ext:.2e})"
        );
        assert!(
            (k_conformed - k_flex).abs() < 0.5 * k_flex.abs(),
            "exact-bone conform shifted k_disc ({k_flex:.1} -> {k_conformed:.1})"
        );

        // (3) A sub-degree sweep (incremental, warm-started) genuinely deforms and
        // restores across angles — conservation + strictly-restoring moment per
        // frame + a real imposed deformation (assert_restoring_sweep, shared with
        // the synthetic and scanned-disc capture tests).
        let angles: Vec<f64> = [-0.5_f64, -0.25, 0.0, 0.25, 0.5]
            .iter()
            .map(|d| d.to_radians())
            .collect();
        let traj = raw_bond.capture_flexion(&angles);
        let max_disp = assert_restoring_sweep(&traj, 2e-2);
        println!(
            "sweep: {} frames, max node displacement {max_disp:.3} mm",
            traj.frames.len()
        );
    }

    /// B6.4 — the capstone: a painted (lofted) disc drops into the FULL coupled
    /// FSU (disc bushing + ligaments + facet contact) and produces a physically
    /// sound, monotone, restoring, facet-stopped segmental response — the same
    /// assembly rung 7 validates against literature, now driven by a human-painted
    /// disc.
    ///
    /// This validates the **integration** (painted disc → full FSU → physiologic
    /// response), not the tight literature ROM band: the rough *auto-lofted* disc
    /// is softer in flexion than the scanned disc (a different centre/geometry
    /// shifts the ligament lever arms), so its flexion exits the ROM bracket
    /// before the full 7.5 N·m. We sweep the moment range where equilibria exist;
    /// a carefully painted disc matching the real geometry is what would hit the
    /// literature band.
    #[test]
    #[ignore = "needs $CF_L4_STL/$CF_L5_STL (BodyParts3D, CC BY-SA, not committed)"]
    fn b6_4_coupled_fsu_from_a_lofted_disc() {
        use cf_fsu_geometry::load_from_env;

        let l4 = load_from_env("CF_L4_STL").unwrap();
        let l5 = load_from_env("CF_L5_STL").unwrap();
        let disc = lofted_disc(&l4, &l5);
        let mut fsu = CoupledFsu::build(&l4, &l5, &disc, &CoupledParams::default())
            .expect("coupled FSU builds with a lofted disc");
        println!("coupled FSU: k_disc {:.3} N·m/rad", fsu.k_disc());

        // Force-driven equilibrium sweep over the in-bracket moment range.
        let ramp: Vec<f64> = (0..=18).map(|i| -6.0 + f64::from(i) * 0.5).collect();
        let traj = fsu
            .capture_ramp(&ramp)
            .expect("coupled equilibria exist within the ROM bracket");

        // Monotone extension → flexion, a physiologic few-degrees ROM each way.
        for w in traj.frames.windows(2) {
            assert!(
                w[1].theta >= w[0].theta - 1e-9,
                "equilibrium angle must rise monotonically with the applied moment"
            );
        }
        let ext_deg = traj.frames.first().unwrap().theta.to_degrees();
        let flex_deg = traj.frames.last().unwrap().theta.to_degrees();
        println!("ROM: extension {ext_deg:.1}° … flexion {flex_deg:.1}° (in-bracket sweep)");
        assert!(ext_deg < 0.0 && flex_deg > 0.0, "wrong flexion sense");

        // The facets stop extension and open in flexion — the ROM is contact-limited,
        // not just spring-limited.
        assert!(
            !traj.frames.first().unwrap().facet_points.is_empty(),
            "facets must engage at the extension peak"
        );
        assert!(
            traj.frames.last().unwrap().facet_points.is_empty(),
            "facets must open at the flexion peak"
        );
    }

    #[test]
    fn ml_axis_is_widest_aabb_extent_not_pca() {
        // The flexion axis must be the widest AXIS-ALIGNED extent, NOT a PCA principal
        // direction. This cloud's longest point-to-point direction is the x-y diagonal, but
        // its widest axis-aligned extent is x (span 10 vs y-span 3) → +x. A regression of
        // `ml_axis_from_points` to a PCA axis would silently rotate the whole segment about
        // the wrong axis on a real oblique disc, with no other test catching it.
        let pts = [Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 3.0, 1.0)];
        assert_eq!(ml_axis_from_points(&pts), Vector3::x());
    }

    #[test]
    fn ml_axis_breaks_extent_ties_by_last_max_not_longest_axis() {
        // The deliberate `max_by` (LAST maximum) tie-break vs `Aabb::longest_axis` (FIRST
        // maximum): x and y extents TIED at the widest must resolve to the last (y). A
        // regression to `longest_axis()` would flip a square disc (a defect the PR-A gating
        // review caught once).
        let pts = [
            Point3::new(-10.0, -10.0, -3.0),
            Point3::new(10.0, 10.0, 3.0),
        ];
        assert_eq!(ml_axis_from_points(&pts), Vector3::y());
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
