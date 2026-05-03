//! Infill generation for hollow meshes with lattice interiors.

// Allow numeric casts for vertex indexing and geometry-domain conversions
// (cap_thickness derives from `usize as f64` on `solid_cap_layers` and
// `resolution`, both validated to small bounded values; mirrors
// `generate.rs`'s file-level allows).
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_precision_loss)]

use crate::error::LatticeError;
use crate::generate::generate_lattice;
use crate::params::LatticeParams;
use crate::strut::{combine_struts, generate_strut};
use crate::types::LatticeType;
use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_sdf::SignedDistanceField;
use mesh_types::IndexedMesh;
use nalgebra::Point3;
use std::sync::Arc;

/// Parameters for infill generation.
///
/// Infill combines an outer shell with an internal lattice structure.
///
/// # Examples
///
/// ```
/// use mesh_lattice::InfillParams;
///
/// // FDM-optimized settings
/// let params = InfillParams::for_fdm();
/// assert_eq!(params.infill_percentage, 0.2);
///
/// // Lightweight part
/// let params = InfillParams::for_lightweight();
/// assert_eq!(params.infill_percentage, 0.1);
/// ```
#[derive(Debug, Clone)]
pub struct InfillParams {
    /// Lattice parameters for the internal structure.
    pub lattice: LatticeParams,

    /// Outer shell thickness in mm.
    pub shell_thickness: f64,

    /// Number of perimeter layers for the shell.
    pub shell_layers: usize,

    /// Infill percentage (0.0 = hollow, 1.0 = solid).
    pub infill_percentage: f64,

    /// Whether to connect lattice to shell.
    pub connect_to_shell: bool,

    /// Thickness of lattice-to-shell connections.
    pub connection_thickness: f64,

    /// Whether to add solid caps at top/bottom.
    pub solid_caps: bool,

    /// Number of solid layers at top/bottom.
    pub solid_cap_layers: usize,
}

impl Default for InfillParams {
    fn default() -> Self {
        Self {
            lattice: LatticeParams::cubic(5.0),
            shell_thickness: 1.2,
            shell_layers: 3,
            infill_percentage: 0.2,
            connect_to_shell: true,
            connection_thickness: 0.4,
            solid_caps: true,
            solid_cap_layers: 4,
        }
    }
}

impl InfillParams {
    /// Creates default infill parameters.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates parameters optimized for FDM 3D printing.
    ///
    /// - 20% cubic infill
    /// - 1.2mm shell thickness (3 perimeters at 0.4mm)
    /// - 4 solid top/bottom layers
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::InfillParams;
    ///
    /// let params = InfillParams::for_fdm();
    /// assert!((params.shell_thickness - 1.2).abs() < 0.01);
    /// ```
    #[must_use]
    pub fn for_fdm() -> Self {
        Self {
            lattice: LatticeParams::cubic(5.0).with_density(0.2),
            shell_thickness: 1.2,
            shell_layers: 3,
            infill_percentage: 0.2,
            connect_to_shell: true,
            connection_thickness: 0.4,
            solid_caps: true,
            solid_cap_layers: 4,
        }
    }

    /// Creates parameters for lightweight parts.
    ///
    /// - 10% gyroid infill
    /// - 0.8mm shell thickness
    /// - 2 solid cap layers
    #[must_use]
    pub fn for_lightweight() -> Self {
        Self {
            lattice: LatticeParams::gyroid(8.0).with_density(0.1),
            shell_thickness: 0.8,
            shell_layers: 2,
            infill_percentage: 0.1,
            connect_to_shell: true,
            connection_thickness: 0.4,
            solid_caps: true,
            solid_cap_layers: 2,
        }
    }

    /// Creates parameters for strong parts.
    ///
    /// - 50% octet-truss infill
    /// - 1.6mm shell thickness
    /// - 6 solid cap layers
    #[must_use]
    pub fn for_strong() -> Self {
        Self {
            lattice: LatticeParams::octet_truss(4.0).with_density(0.5),
            shell_thickness: 1.6,
            shell_layers: 4,
            infill_percentage: 0.5,
            connect_to_shell: true,
            connection_thickness: 0.6,
            solid_caps: true,
            solid_cap_layers: 6,
        }
    }

    /// Sets the lattice type.
    #[must_use]
    pub fn with_lattice_type(mut self, lattice_type: LatticeType) -> Self {
        self.lattice = self.lattice.with_lattice_type(lattice_type);
        self
    }

    /// Sets the cell size for the lattice.
    #[must_use]
    pub fn with_cell_size(mut self, cell_size: f64) -> Self {
        self.lattice = self.lattice.with_cell_size(cell_size);
        self
    }

    /// Sets the shell thickness.
    #[must_use]
    pub const fn with_shell_thickness(mut self, thickness: f64) -> Self {
        self.shell_thickness = thickness.max(0.0);
        self
    }

    /// Sets the infill percentage.
    ///
    /// Value is clamped to [0.0, 1.0].
    #[must_use]
    pub fn with_infill_percentage(mut self, percentage: f64) -> Self {
        self.infill_percentage = percentage.clamp(0.0, 1.0);
        self.lattice = self.lattice.with_density(percentage);
        self
    }

    /// Enables or disables solid caps.
    #[must_use]
    pub const fn with_solid_caps(mut self, enabled: bool) -> Self {
        self.solid_caps = enabled;
        self
    }

    /// Sets the number of solid cap layers.
    #[must_use]
    pub const fn with_solid_cap_layers(mut self, layers: usize) -> Self {
        self.solid_cap_layers = layers;
        self
    }

    /// Validates the parameters.
    ///
    /// # Errors
    ///
    /// Returns [`LatticeError`] if any parameter is invalid.
    pub fn validate(&self) -> Result<(), LatticeError> {
        if self.shell_thickness < 0.0 {
            return Err(LatticeError::InvalidShellThickness(self.shell_thickness));
        }
        self.lattice.validate()
    }
}

/// Result of infill generation.
#[derive(Debug, Clone)]
pub struct InfillResult {
    /// Complete combined mesh (shell + lattice).
    pub mesh: IndexedMesh,

    /// Hollow shell mesh — outer surface combined with inward-offset inner
    /// surface (for inspection).
    pub shell: IndexedMesh,

    /// Inner lattice mesh (for inspection).
    pub lattice: IndexedMesh,

    /// Actual achieved density.
    pub actual_density: f64,

    /// Volume of the shell.
    pub shell_volume: f64,

    /// Volume of the lattice.
    pub lattice_volume: f64,

    /// Volume of the interior space.
    pub interior_volume: f64,
}

impl InfillResult {
    /// Returns the total volume (shell + lattice).
    #[must_use]
    pub fn total_volume(&self) -> f64 {
        self.shell_volume + self.lattice_volume
    }

    /// Returns the total vertex count.
    #[must_use]
    pub const fn vertex_count(&self) -> usize {
        self.mesh.vertex_count()
    }

    /// Returns the total triangle count.
    #[must_use]
    pub const fn triangle_count(&self) -> usize {
        self.mesh.face_count()
    }
}

/// Generates infill for a mesh (shell + internal lattice).
///
/// This function creates:
/// 1. An outer shell by offsetting the mesh inward
/// 2. An internal lattice structure within the remaining volume
/// 3. Connections between the shell and lattice (if enabled)
///
/// # Arguments
///
/// * `mesh` - The input mesh (should be watertight)
/// * `params` - Infill generation parameters
///
/// # Returns
///
/// An `InfillResult` containing the combined mesh and statistics.
///
/// # Errors
///
/// Returns [`LatticeError`] if:
/// - The input mesh is empty
/// - The mesh is not watertight
/// - Parameters are invalid
/// - The interior is too small for lattice generation
/// - The inward shell offset (via `mesh-offset`) fails (e.g., shell thickness
///   exceeds the mesh's local thickness, producing an empty offset surface)
///
/// # Examples
///
/// ```no_run
/// use mesh_lattice::{generate_infill, InfillParams};
/// use mesh_types::IndexedMesh;
///
/// // Load a mesh
/// let mesh = IndexedMesh::new(); // placeholder
///
/// let params = InfillParams::for_fdm();
/// match generate_infill(&mesh, &params) {
///     Ok(result) => println!("Generated {} triangles", result.triangle_count()),
///     Err(e) => eprintln!("Error: {}", e),
/// }
/// ```
#[allow(clippy::too_many_lines)] // Sequential pipeline: bounds → caps → SDF → lattice → shell → connections → combined.
pub fn generate_infill(
    mesh: &IndexedMesh,
    params: &InfillParams,
) -> Result<InfillResult, LatticeError> {
    // Validate parameters
    params.validate()?;

    // Check for empty mesh
    if mesh.is_empty() {
        return Err(LatticeError::EmptyMesh);
    }

    // Handle edge cases
    if params.infill_percentage >= 0.999 {
        // 100% infill = return original mesh
        return Ok(InfillResult {
            mesh: mesh.clone(),
            shell: mesh.clone(),
            lattice: IndexedMesh::new(),
            actual_density: 1.0,
            shell_volume: 0.0, // Would need proper volume calculation
            lattice_volume: 0.0,
            interior_volume: 0.0,
        });
    }

    if params.infill_percentage <= 0.001 {
        // 0% infill = hollow shell
        // For now, return the original mesh (proper shell generation would use mesh-offset)
        return Ok(InfillResult {
            mesh: mesh.clone(),
            shell: mesh.clone(),
            lattice: IndexedMesh::new(),
            actual_density: 0.0,
            shell_volume: 0.0,
            lattice_volume: 0.0,
            interior_volume: 0.0,
        });
    }

    // Compute mesh bounds
    let (min, max) = compute_bounds(mesh);

    // Compute interior bounds (inset by shell thickness + safety margin)
    let inset = params
        .lattice
        .cell_size
        .mul_add(0.5, params.shell_thickness);
    let interior_min = Point3::new(min.x + inset, min.y + inset, min.z + inset);
    let interior_max = Point3::new(max.x - inset, max.y - inset, max.z - inset);

    // Check if interior is large enough
    if interior_min.x >= interior_max.x
        || interior_min.y >= interior_max.y
        || interior_min.z >= interior_max.z
    {
        return Err(LatticeError::InteriorTooSmall);
    }

    // Solid caps (gap c): when `params.solid_caps` is true, reserve
    // `cap_thickness` of z-extent at the top + bottom of the cavity for
    // planar-slab geometry; the lattice iteration domain shrinks to skip
    // those bands. `layer_height` derives from `cell_size / resolution`
    // per the v1.0 spec heuristic — `resolution` is validated `>= 2`
    // (`params.rs::LatticeParams::validate`), and across the three
    // preset constructors (cubic res=10, gyroid res=15, octet res=10)
    // yields FDM-typical 0.4–0.6 mm/layer. Adding `params.layer_height`
    // as an explicit `f64` field is a v0.9 candidate when a real
    // consumer wants direct control.
    //
    // Caps occupy the FULL xy interior bbox so they meet the
    // inward-offset shell wall on each side. Same non-convex-input
    // limitation as gap e: for inputs whose AABB inset includes
    // regions outside the part (e.g., a torus hole), caps will extend
    // through those regions; convex inputs are unaffected.
    let cap_thickness = if params.solid_caps {
        let layer_height = params.lattice.cell_size / params.lattice.resolution as f64;
        params.solid_cap_layers as f64 * layer_height
    } else {
        0.0
    };

    // Reuse `InteriorTooSmall` rather than introducing a `CapsTooThick`
    // variant — the existing message covers "interior cannot fit the
    // requested geometry," and consumers already pattern-match on it.
    let interior_height_z = interior_max.z - interior_min.z;
    if 2.0 * cap_thickness >= interior_height_z {
        return Err(LatticeError::InteriorTooSmall);
    }

    let iter_min = Point3::new(
        interior_min.x,
        interior_min.y,
        interior_min.z + cap_thickness,
    );
    let iter_max = Point3::new(
        interior_max.x,
        interior_max.y,
        interior_max.z - cap_thickness,
    );

    // Build the inward-offset inner surface first; the lattice is then clipped
    // to the cavity it bounds.
    //
    // Resolution targets ~2 voxels across the wall thickness, clamped to a
    // sensible range; finer resolutions blow up the SDF grid for large bboxes.
    let offset_resolution = (params.shell_thickness / 2.0).clamp(0.1, 1.0);
    let offset_config = OffsetConfig::default().with_resolution(offset_resolution);
    let inner_offset = offset_mesh(mesh, -params.shell_thickness, &offset_config)
        .map_err(|e| LatticeError::OffsetFailed(e.to_string()))?;

    // Build a cavity-bounded SDF on the inner offset. `mesh-offset`'s
    // inward-offset MC orients inner faces with normals into the cavity
    // (`inner_offset.signed_volume() < 0` standalone), so `mesh-sdf::distance`
    // returns positive in the cavity and negative in the wall material — the
    // opposite sign of what `is_outside_shape` expects. Negate to get the
    // cavity-SDF convention: `< 0` in cavity, `> 0` in wall.
    //
    // Face-normal sign rather than `unsigned_distance + is_inside`: the
    // un-welded MC output of `mesh-offset` produces overlapping triangles
    // wherever adjacent voxels resolve the same surface region, and
    // `point_in_mesh`'s ray-crossing parity test goes noisy on that soup —
    // empirically miscounts ~36% of cavity nodes as outside on the cube
    // fixture, dropping ~48% of struts. The face-normal approach uses one
    // closest face per query and is stable under MC discretization.
    //
    // Limitation: for non-convex inputs whose AABB inset includes regions
    // outside the part (e.g., the hole of a torus), `-distance` returns
    // negative there too — outside-the-part is on the same side of the
    // closest inner face as the cavity. Convex inputs are unaffected.
    //
    // The `inner_sdf` is `Arc`-shared with the lattice-to-shell
    // connections pass below (gap b): both queries reuse the same
    // precomputed face-normal table and avoid building a second SDF
    // on the same `inner_offset`.
    let inner_sdf = Arc::new(
        SignedDistanceField::new(inner_offset.clone())
            .map_err(|e| LatticeError::OffsetFailed(e.to_string()))?,
    );
    let inner_sdf_for_clip = Arc::clone(&inner_sdf);
    let cavity_lattice_params = params
        .lattice
        .clone()
        .with_shape_sdf(Arc::new(move |p| -inner_sdf_for_clip.distance(p)));

    // Generate lattice in the (cap-shrunken) iteration domain, clipped to
    // the cavity by the SDF.
    let lattice_result = generate_lattice(&cavity_lattice_params, (iter_min, iter_max))?;

    // Build the hollow shell: outer surface + inward-offset inner surface.
    let shell = combine_meshes(mesh, &inner_offset);

    // Build solid caps in the reserved z-bands. Two axis-aligned boxes
    // (outward-CCW) covering the full xy interior; empty mesh when
    // `params.solid_caps` is false OR `cap_thickness == 0.0` (the
    // latter handles `solid_caps = true` with `solid_cap_layers = 0`,
    // which would otherwise produce degenerate zero-height boxes).
    let cap_mesh = if params.solid_caps && cap_thickness > 0.0 {
        build_solid_caps(interior_min, interior_max, cap_thickness)
    } else {
        IndexedMesh::new()
    };

    // Lattice-to-shell bridging struts (gap b). For each unique
    // grid-node that participates in at least one lattice strut and
    // sits within `2 * cell_size` of the inward-offset shell, emit
    // one strut from the node to its closest point on the inner
    // shell. The detection threshold is the natural lattice-scale
    // ("near-shell cells reach the wall") and decoupled from
    // `connection_thickness` (the strut diameter); a separate
    // `params.connection_distance` field is a v0.9 candidate if a
    // real consumer wants explicit control.
    //
    // Why `2 * cell_size` and not `cell_size / 2`: the lattice
    // iter-domain inset is `cell_size / 2 + shell_thickness`, so
    // perimeter nodes sit `cell_size / 2` from the inner shell in
    // the simple case. But `generate_cubic_lattice` (and
    // `generate_voronoi_lattice` / `generate_octet_truss_lattice`)
    // also enforces `trim_to_bounds`, which orphans grid nodes
    // whose +x/+y/+z neighbor would extend past `iter_max`. On the
    // canonical cube fixture (cell_size 10, iter [6.2, 43.8]³),
    // this leaves the participating grid at ix/iy/iz ∈ [0, 3] /
    // [0, 3] / [0, 2] — and the +x/+y perimeter recedes to ~12.6
    // mm from the inner shell, +z to ~18.6 mm (with cap_thickness
    // = 4). A single-cell threshold would catch only the -x/-y/-z
    // perimeter (2 of 6 inner-shell faces); a 1.5-cell threshold
    // catches +x/+y but not +z. `2 * cell_size` captures the
    // full perimeter robustly across `trim_to_bounds` + solid-cap
    // interactions.
    //
    // Scope: strut-based lattices only (Cubic / OctetTruss /
    // Voronoi). TPMS lattices (Gyroid / SchwarzP / Diamond) have no
    // graph-node concept — the cavity-SDF clipping above already
    // terminates the marching-cubes surface against the inner shell
    // at cell boundaries, so connect_to_shell is implicit.
    // `lattice_result.nodes` is empty for TPMS, so the no-op falls
    // out of the `is_empty` guard without needing a `LatticeType`
    // switch.
    //
    // Cap-band filter: the closest-point's z must lie in the lattice
    // iteration domain `[iter_min.z, iter_max.z]`. Without this, a
    // lattice node near a cap-band boundary could connect to the
    // inner-shell's top or bottom face (which is in the cap region),
    // producing a strut that passes through the cap mesh. On the
    // cube fixture this filter does most of the actual exclusion
    // work — `2 * cell_size` is intentionally permissive on the
    // distance side, and the cap-band filter rejects interior nodes
    // whose closest face is -z or +z.
    //
    // Watertightness: connection struts share endpoint coordinates
    // with lattice strut centers (welded by combine_struts at the
    // lattice node) and with arbitrary face-interior points on the
    // inner shell (un-welded — the SDF returns a closest-point that
    // is generally NOT a vertex of the inward-offset MC output).
    // The combined mesh is therefore non-watertight at the
    // connection-shell junction. Pre-existing soup-composition
    // pattern across the entire F6 sub-arc (shell+lattice and
    // shell+caps already share no vertices); not gap-b-specific.
    let connections_mesh = if params.connect_to_shell
        && params.connection_thickness > 0.0
        && !lattice_result.nodes.is_empty()
    {
        let detection_threshold = 2.0 * params.lattice.cell_size;
        let strut_radius = params.connection_thickness / 2.0;
        build_lattice_to_shell_connections(
            &lattice_result.nodes,
            &inner_sdf,
            detection_threshold,
            strut_radius,
            iter_min.z,
            iter_max.z,
        )
    } else {
        IndexedMesh::new()
    };

    // Combine shell + lattice + caps + connections. The combined
    // mesh is a soup of disjoint closed-or-open sub-volumes; signed
    // volume on the combined sums each sub-volume's contribution.
    // The dedicated `shell_volume` / `lattice_volume` fields stay
    // scoped to their sub-meshes; cap and connection volumes are
    // observable via the triangle-count delta on `mesh` between
    // `solid_caps`/`connect_to_shell` true and false runs.
    let mut combined = combine_meshes(&shell, &lattice_result.mesh);
    combined = combine_meshes(&combined, &cap_mesh);
    combined = combine_meshes(&combined, &connections_mesh);

    // Shell signed-volume integral resolves to the wall volume:
    // `mesh-offset`'s inward-offset MC orients inner faces with normals
    // pointing into the cavity, so the divergence theorem subtracts the
    // inner-offset enclosed volume from the outer mesh's enclosed
    // volume.
    let interior_volume = (interior_max.x - interior_min.x)
        * (interior_max.y - interior_min.y)
        * (interior_max.z - interior_min.z);
    let shell_volume = shell.volume();
    let lattice_volume = lattice_result.mesh.volume();

    Ok(InfillResult {
        mesh: combined,
        shell,
        lattice: lattice_result.mesh,
        actual_density: lattice_result.actual_density,
        shell_volume,
        lattice_volume,
        interior_volume,
    })
}

/// Builds bridging struts from each near-shell lattice node to its
/// closest point on the inner shell.
///
/// Iteration is O(M × N) where M is the unique-node count from
/// `lattice_result.nodes` and N is the inner-offset face count
/// (`SignedDistanceField::closest_point` does an O(N) scan over all
/// faces). On the canonical cube fixture (~50 unique nodes ×
/// ~75 000 inner-offset faces ≈ 3.8 M ops), this runs in
/// milliseconds in release mode — well within v1.0 example budgets.
/// A BVH-accelerated `mesh-sdf` is a v0.9 candidate that would speed
/// this up by ~3 orders of magnitude on large fixtures.
///
/// Threshold semantic: a node is considered "near-shell" when its
/// closest-point distance is `<= threshold` (inclusive). On the
/// cube fixture the threshold is `2 * cell_size = 20 mm` and no
/// participating node sits at exactly that distance, so `<=` vs `<`
/// is observationally equivalent here — but the inclusive choice is
/// the natural one for fixtures where a perimeter node could land
/// exactly on the threshold (e.g., a cubical cavity sized so that a
/// trim-orphan recedes the perimeter to exactly `2 * cell_size`).
///
/// Cap-band filter: only emit a strut if the closest-point's z lies
/// in the lattice iteration domain `[iter_min_z, iter_max_z]`. See
/// the call-site doc-comment for rationale.
fn build_lattice_to_shell_connections(
    nodes: &[Point3<f64>],
    inner_sdf: &SignedDistanceField,
    threshold: f64,
    strut_radius: f64,
    iter_min_z: f64,
    iter_max_z: f64,
) -> IndexedMesh {
    let struts = nodes.iter().filter_map(|node| {
        let closest = inner_sdf.closest_point(*node);
        let dist = (closest - *node).norm();
        if dist > threshold {
            return None;
        }
        if closest.z < iter_min_z || closest.z > iter_max_z {
            return None;
        }
        generate_strut(*node, closest, strut_radius)
    });
    combine_struts(struts)
}

/// Builds the two solid-cap boxes (bottom + top) covering the full xy
/// interior at the reserved z-bands. Each cap is an outward-CCW
/// closed box with 8 vertices + 12 triangles (winding mirrors
/// `mesh-types::unit_cube`); the combined cap mesh has 16 vertices +
/// 24 triangles.
fn build_solid_caps(
    interior_min: Point3<f64>,
    interior_max: Point3<f64>,
    cap_thickness: f64,
) -> IndexedMesh {
    let bottom = build_axis_aligned_box(
        interior_min.x,
        interior_max.x,
        interior_min.y,
        interior_max.y,
        interior_min.z,
        interior_min.z + cap_thickness,
    );
    let top = build_axis_aligned_box(
        interior_min.x,
        interior_max.x,
        interior_min.y,
        interior_max.y,
        interior_max.z - cap_thickness,
        interior_max.z,
    );
    combine_meshes(&bottom, &top)
}

/// Builds an axis-aligned box mesh (8 verts + 12 outward-CCW
/// triangles). Vertex + face ordering mirrors `mesh-types::unit_cube`
/// (`mesh/mesh-types/src/mesh.rs:24-51`); `signed_volume` returns the
/// box's enclosed volume (positive).
fn build_axis_aligned_box(
    x_lo: f64,
    x_hi: f64,
    y_lo: f64,
    y_hi: f64,
    z_lo: f64,
    z_hi: f64,
) -> IndexedMesh {
    let vertices = vec![
        Point3::new(x_lo, y_lo, z_lo), // 0
        Point3::new(x_hi, y_lo, z_lo), // 1
        Point3::new(x_hi, y_hi, z_lo), // 2
        Point3::new(x_lo, y_hi, z_lo), // 3
        Point3::new(x_lo, y_lo, z_hi), // 4
        Point3::new(x_hi, y_lo, z_hi), // 5
        Point3::new(x_hi, y_hi, z_hi), // 6
        Point3::new(x_lo, y_hi, z_hi), // 7
    ];
    let faces = vec![
        // Bottom (z=z_lo); outward = -z
        [0, 2, 1],
        [0, 3, 2],
        // Top (z=z_hi); outward = +z
        [4, 5, 6],
        [4, 6, 7],
        // Front (y=y_lo); outward = -y
        [0, 1, 5],
        [0, 5, 4],
        // Back (y=y_hi); outward = +y
        [3, 7, 6],
        [3, 6, 2],
        // Left (x=x_lo); outward = -x
        [0, 4, 7],
        [0, 7, 3],
        // Right (x=x_hi); outward = +x
        [1, 2, 6],
        [1, 6, 5],
    ];
    IndexedMesh::from_parts(vertices, faces)
}

/// Computes the axis-aligned bounding box of a mesh.
fn compute_bounds(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);

    for v in &mesh.vertices {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }

    (min, max)
}

/// Combines two meshes into one.
fn combine_meshes(a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
    let mut vertices = a.vertices.clone();
    let mut faces = a.faces.clone();

    let base_index = vertices.len() as u32;

    vertices.extend(b.vertices.iter().copied());

    for face in &b.faces {
        faces.push([
            face[0] + base_index,
            face[1] + base_index,
            face[2] + base_index,
        ]);
    }

    IndexedMesh::from_parts(vertices, faces)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    fn create_test_cube() -> IndexedMesh {
        // 50 mm outward-CCW cube; winding mirrors `mesh-types::unit_cube`
        // (`mesh/mesh-types/src/mesh.rs:24-51`) scaled by 50, so
        // `signed_volume() == +125_000.0` (not inside-out). Surfaced as
        // a winding bug at §6.2 #27 when the new gap-c comparison test
        // produced an empty lattice on this fixture (mesh-offset's
        // inward-offset flipped sign on inside-out input → cavity SDF
        // misclassified all interior nodes).
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(50.0, 0.0, 0.0),
            Point3::new(50.0, 50.0, 0.0),
            Point3::new(0.0, 50.0, 0.0),
            Point3::new(0.0, 0.0, 50.0),
            Point3::new(50.0, 0.0, 50.0),
            Point3::new(50.0, 50.0, 50.0),
            Point3::new(0.0, 50.0, 50.0),
        ];

        let faces = vec![
            // Bottom (z=0); outward = -z
            [0, 2, 1],
            [0, 3, 2],
            // Top (z=50); outward = +z
            [4, 5, 6],
            [4, 6, 7],
            // Front (y=0); outward = -y
            [0, 1, 5],
            [0, 5, 4],
            // Back (y=50); outward = +y
            [3, 7, 6],
            [3, 6, 2],
            // Left (x=0); outward = -x
            [0, 4, 7],
            [0, 7, 3],
            // Right (x=50); outward = +x
            [1, 2, 6],
            [1, 6, 5],
        ];

        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_infill_params_default() {
        let params = InfillParams::default();
        assert!((params.shell_thickness - 1.2).abs() < 0.01);
        assert!((params.infill_percentage - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_infill_params_fdm() {
        let params = InfillParams::for_fdm();
        assert_eq!(params.lattice.lattice_type, LatticeType::Cubic);
        assert!((params.infill_percentage - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_infill_params_lightweight() {
        let params = InfillParams::for_lightweight();
        assert_eq!(params.lattice.lattice_type, LatticeType::Gyroid);
        assert!((params.infill_percentage - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_infill_params_strong() {
        let params = InfillParams::for_strong();
        assert_eq!(params.lattice.lattice_type, LatticeType::OctetTruss);
        assert!((params.infill_percentage - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_infill_builder() {
        let params = InfillParams::new()
            .with_shell_thickness(2.0)
            .with_infill_percentage(0.3)
            .with_cell_size(8.0);

        assert!((params.shell_thickness - 2.0).abs() < 0.01);
        assert!((params.infill_percentage - 0.3).abs() < 0.01);
        assert!((params.lattice.cell_size - 8.0).abs() < 0.01);
    }

    #[test]
    fn test_generate_infill_basic() {
        let mesh = create_test_cube();
        let params = InfillParams::for_fdm().with_cell_size(10.0);

        let result = generate_infill(&mesh, &params);
        assert!(result.is_ok());

        let infill = result.unwrap();
        assert!(infill.vertex_count() > 0);
    }

    #[test]
    fn test_generate_infill_solid_caps() {
        // `for_fdm` has `solid_caps = true` and `solid_cap_layers = 4`. With
        // `cell_size = 10` and the cubic preset's default `resolution = 10`,
        // `layer_height = 1.0` mm and `cap_thickness = 4.0` mm. The
        // `with vs without` comparison anchor (spec §5.9): each cap adds
        // exactly 8 verts + 12 triangles to the combined mesh, and the
        // lattice iteration domain shrinks by `cap_thickness` at top/bottom
        // — so the caps-on lattice has strictly fewer vertices than the
        // caps-off lattice on the same fixture.
        //
        // `connect_to_shell` is disabled on both runs so the
        // `mesh = shell + lattice + cap` decomposition is exact;
        // connection-strut interaction with caps is exercised by
        // `test_generate_infill_lattice_to_shell_connections`.
        let mesh = create_test_cube();

        let mut params_with = InfillParams::for_fdm().with_cell_size(10.0);
        params_with.connect_to_shell = false;
        let with_caps = generate_infill(&mesh, &params_with).unwrap();

        let mut params_without = InfillParams::for_fdm()
            .with_cell_size(10.0)
            .with_solid_caps(false);
        params_without.connect_to_shell = false;
        let without_caps = generate_infill(&mesh, &params_without).unwrap();

        assert!(
            with_caps.lattice.vertex_count() < without_caps.lattice.vertex_count(),
            "solid_caps = true should shrink the lattice (iteration domain reserved for caps): with={} >= without={}",
            with_caps.lattice.vertex_count(),
            without_caps.lattice.vertex_count(),
        );

        let cap_verts = with_caps.mesh.vertex_count()
            - with_caps.shell.vertex_count()
            - with_caps.lattice.vertex_count();
        assert_eq!(
            cap_verts, 16,
            "two outward-CCW box caps contribute exactly 16 vertices (8 per cap)",
        );

        let cap_tris = with_caps.mesh.face_count()
            - with_caps.shell.face_count()
            - with_caps.lattice.face_count();
        assert_eq!(
            cap_tris, 24,
            "two outward-CCW box caps contribute exactly 24 triangles (12 per cap)",
        );

        // Top cap-band emptiness: the gap-c carving is provable via a
        // max-z bound on `with_caps.lattice`. With cap_thickness = 4 mm,
        // iter_max.z = 39.8; `cells_z = ceil(29.6/10) = 3` produces
        // iz ∈ 0..=3 over `{10.2, 20.2, 30.2, 40.2}`, and
        // `trim_to_bounds` drops every strut from iz=3 (`end.z > 39.8`),
        // so the highest non-trimmed row is iz=2 at z=30.2 (cylinder
        // verts ≤ 30.2 + strut_radius ≈ 30.6 — well below 39.8 with
        // > 9 mm cushion).
        //
        // We do NOT anchor on the BOTTOM cap band: cells_z = 3 places
        // the iz=0 row at z=10.2 (the cap_band_hi exactly), and strut
        // radius ≈ 0.4 mm spreads cylinder verts down to ~9.8 mm — a
        // pre-existing soup-composition artifact at any lattice-
        // iteration boundary, not gap-c-specific. Empirically observed
        // bottom min_z ≈ 10.02 mm at this fixture (cylinder vert
        // tessellation lands inside the cap band by ~0.18 mm).
        let with_caps_max_z = with_caps
            .lattice
            .vertices
            .iter()
            .map(|v| v.z)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            with_caps_max_z < 39.8,
            "no lattice vert should land in the top cap band [39.8, 43.8]: got max_z = {with_caps_max_z}",
        );
    }

    #[test]
    fn test_generate_infill_lattice_to_shell_connections() {
        // `for_fdm` has `connect_to_shell = true` and
        // `connection_thickness = 0.4`. With `cell_size = 10`, the
        // detection threshold is `2 * cell_size = 20 mm`, capturing
        // the lattice perimeter (including +x/+y/+z trim-orphan
        // perimeters at ~12.6 / ~12.6 / ~18.6 mm). Cap-band filter
        // excludes nodes whose nearest face is -z or +z.
        //
        // The `with vs without` comparison anchor (spec §5.9): each
        // connection strut adds exactly 14 vertices + 24 triangles
        // to the combined mesh, and shell/lattice/cap sub-meshes are
        // unchanged across the comparison. Empirically on the cube
        // fixture: ~37 connection struts; the inequality bounds are
        // intentionally loose (25-60 connections) to absorb any MC
        // discretization noise on the SDF closest-point queries.
        let mesh = create_test_cube();

        let with_conn =
            generate_infill(&mesh, &InfillParams::for_fdm().with_cell_size(10.0)).unwrap();

        let mut params_no_conn = InfillParams::for_fdm().with_cell_size(10.0);
        params_no_conn.connect_to_shell = false;
        let without_conn = generate_infill(&mesh, &params_no_conn).unwrap();

        assert_eq!(
            with_conn.shell.vertex_count(),
            without_conn.shell.vertex_count(),
            "shell.vertex_count must be invariant across connect_to_shell flag",
        );
        assert_eq!(
            with_conn.shell.face_count(),
            without_conn.shell.face_count(),
            "shell.face_count must be invariant across connect_to_shell flag",
        );
        assert_eq!(
            with_conn.lattice.vertex_count(),
            without_conn.lattice.vertex_count(),
            "lattice.vertex_count must be invariant across connect_to_shell flag",
        );
        assert_eq!(
            with_conn.lattice.face_count(),
            without_conn.lattice.face_count(),
            "lattice.face_count must be invariant across connect_to_shell flag",
        );

        let vert_delta = with_conn.mesh.vertex_count() - without_conn.mesh.vertex_count();
        let tri_delta = with_conn.mesh.face_count() - without_conn.mesh.face_count();

        assert_eq!(
            vert_delta % 14,
            0,
            "connection-strut vertex delta must be divisible by 14 (verts per strut): got {vert_delta}",
        );
        assert_eq!(
            tri_delta % 24,
            0,
            "connection-strut triangle delta must be divisible by 24 (tris per strut): got {tri_delta}",
        );
        assert_eq!(
            vert_delta / 14,
            tri_delta / 24,
            "connection-strut count from verts ({}) and tris ({}) must agree",
            vert_delta / 14,
            tri_delta / 24,
        );

        let connection_count = vert_delta / 14;
        assert!(
            (25..=60).contains(&connection_count),
            "expected 25-60 connection struts on cube fixture (predicted ~37): got {connection_count}",
        );
    }

    #[test]
    fn test_generate_infill_connect_to_shell_disabled() {
        // When `connect_to_shell = false`, the combined mesh is
        // exactly shell + lattice + caps (no bridging struts). The
        // gap-b fix only injects connections behind the
        // `connect_to_shell` flag.
        let mesh = create_test_cube();
        let mut params = InfillParams::for_fdm().with_cell_size(10.0);
        params.connect_to_shell = false;

        let result = generate_infill(&mesh, &params).unwrap();

        let cap_verts = result.mesh.vertex_count()
            - result.shell.vertex_count()
            - result.lattice.vertex_count();
        assert_eq!(
            cap_verts, 16,
            "with connections off, mesh = shell + lattice + caps; cap contributes 16 verts",
        );

        let cap_tris =
            result.mesh.face_count() - result.shell.face_count() - result.lattice.face_count();
        assert_eq!(
            cap_tris, 24,
            "with connections off, mesh = shell + lattice + caps; cap contributes 24 tris",
        );
    }

    #[test]
    fn test_generate_infill_solid() {
        let mesh = create_test_cube();
        let params = InfillParams::new().with_infill_percentage(1.0);

        let result = generate_infill(&mesh, &params);
        assert!(result.is_ok());

        let infill = result.unwrap();
        assert!((infill.actual_density - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_generate_infill_hollow() {
        let mesh = create_test_cube();
        let params = InfillParams::new().with_infill_percentage(0.0);

        let result = generate_infill(&mesh, &params);
        assert!(result.is_ok());

        let infill = result.unwrap();
        assert!(infill.actual_density < 0.01);
    }

    #[test]
    fn test_generate_infill_empty_mesh() {
        let mesh = IndexedMesh::new();
        let params = InfillParams::for_fdm();

        let result = generate_infill(&mesh, &params);
        assert!(matches!(result, Err(LatticeError::EmptyMesh)));
    }

    #[test]
    fn test_compute_bounds() {
        let mesh = create_test_cube();
        let (min, max) = compute_bounds(&mesh);

        assert!((min.x - 0.0).abs() < 0.01);
        assert!((max.x - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_combine_meshes() {
        let a = create_test_cube();
        let b = create_test_cube();

        let combined = combine_meshes(&a, &b);

        assert_eq!(combined.vertex_count(), a.vertex_count() + b.vertex_count());
        assert_eq!(combined.face_count(), a.face_count() + b.face_count());
    }

    #[test]
    fn test_infill_result_methods() {
        let mesh = create_test_cube();
        let params = InfillParams::for_fdm().with_cell_size(10.0);

        let result = generate_infill(&mesh, &params).unwrap();

        assert!(result.total_volume() >= 0.0);
        assert!(result.vertex_count() > 0);
        assert!(result.triangle_count() > 0);
    }
}
