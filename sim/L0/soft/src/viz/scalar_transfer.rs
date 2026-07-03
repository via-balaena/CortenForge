//! Shared scalar-transfer spine: per-tet -> per-display-vertex sampling
//! (volume-weighted per-vertex projection, barycentric point-in-tet, kNN-IDW,
//! nearest-tet-centroid) plus the `TetGrid` spatial index and per-tet validation.
//! Used by both the analysis-mesh (F1) and design-mesh (F2) primitives.

use std::collections::BTreeMap;

use nalgebra::Matrix3;

use crate::Vec3;
use crate::material::Material;
use crate::mesh::Mesh;

use super::error::VizError;

/// C2.2 IDW hyperparameter — top-k count for the continuous-scalar
/// kNN-IDW sampler [`idw_k_nearest_tet_centroids`].
///
/// k = 8 matches the typical tets-per-cell on a BCC analysis mesh
/// (six BCC tets per cubic cell, plus boundary-overflow from the
/// 3×3×3 `TetGrid` window) — the smoothing kernel is effectively
/// "one Voronoi neighborhood." Smaller k narrows the smoothing
/// (k = 1 collapses to nearest-tet, identical to the C2.1 categorical
/// path); larger k widens it but admits centroids from further away
/// that aren't physically relevant to the probe.
pub(super) const IDW_K: usize = 8;

/// Find the analysis-mesh tet enclosing `probe` and return barycentric-
/// interpolated per-vertex scalars; falls back to nearest-tet (clipped
/// + renormalized barycentrics) for points just outside every tet.
//
// `cast_possible_truncation` allowed for VertexId casts (u32 crate-wide).
#[allow(clippy::cast_possible_truncation, clippy::similar_names)]
pub(super) fn sample_analysis_at_point<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
    per_vertex_by_scalar: &[Vec<f64>],
) -> Vec<f64> {
    // Tolerance is dimensionless (barycentric coordinates sum to 1.0);
    // -1e-9 forgives pure floating-point round-off without admitting
    // extrapolation into neighbouring tets.
    let tol_bary = -1e-9_f64;
    let mut best_outside_t: Option<u32> = None;
    let mut best_outside_min_b = f64::NEG_INFINITY;

    for &t in candidate_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        if m.determinant().abs() < 1e-30 {
            continue;
        }
        let Some(inv) = m.try_inverse() else {
            continue;
        };
        let b = inv * (probe - p0);
        let b0 = 1.0 - b.x - b.y - b.z;
        let bs = [b0, b.x, b.y, b.z];
        let min_b = bs.iter().copied().fold(f64::INFINITY, f64::min);
        if min_b >= tol_bary {
            return per_vertex_by_scalar
                .iter()
                .map(|pv| {
                    bs[3].mul_add(
                        pv[v3 as usize],
                        bs[2].mul_add(
                            pv[v2 as usize],
                            bs[1].mul_add(pv[v1 as usize], bs[0] * pv[v0 as usize]),
                        ),
                    )
                })
                .collect();
        }
        if min_b > best_outside_min_b {
            best_outside_min_b = min_b;
            best_outside_t = Some(t);
        }
    }

    if let Some(t) = best_outside_t {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        if let Some(inv) = m.try_inverse() {
            let b = inv * (probe - p0);
            let b0 = 1.0 - b.x - b.y - b.z;
            let mut bs = [b0.max(0.0), b.x.max(0.0), b.y.max(0.0), b.z.max(0.0)];
            let sum: f64 = bs.iter().sum();
            if sum > 0.0 {
                for v in &mut bs {
                    *v /= sum;
                }
                return per_vertex_by_scalar
                    .iter()
                    .map(|pv| {
                        bs[3].mul_add(
                            pv[v3 as usize],
                            bs[2].mul_add(
                                pv[v2 as usize],
                                bs[1].mul_add(pv[v1 as usize], bs[0] * pv[v0 as usize]),
                            ),
                        )
                    })
                    .collect();
            }
        }
    }
    per_vertex_by_scalar.iter().map(|_| 0.0).collect()
}

/// Uniform 3D grid of analysis-tet IDs, keyed by tet AABB intersection.
///
/// Cuts the per-probe scalar lookup from `O(n_tets)` to ~27 ×
/// `tets_per_cell`, where `tets_per_cell` is small (~3-5 on row-20-
/// shaped BCC + IS meshes at the empirically-tuned `avg_tet_edge ×
/// 0.25` cell size — see [`TetGrid::build`] for the cell-size choice
/// rationale).
///
/// Built once per [`design_surface`] call; cost is `O(n_tets × cells_
/// per_tet)` to register each tet in every cell its AABB overlaps.
pub(super) struct TetGrid {
    /// Grid origin (mesh-AABB minimum corner, lightly padded).
    origin: Vec3,
    /// Uniform cell side length in world units.
    pub(super) cell_size: f64,
    /// Cells along each axis.
    nx: usize,
    ny: usize,
    nz: usize,
    /// Flat-indexed cell storage: `cells[iz * nx * ny + iy * nx + ix]`
    /// holds the tet IDs whose AABB intersects that cell.
    cells: Vec<Vec<u32>>,
}

impl TetGrid {
    /// Build a grid from `mesh`'s tet AABBs.
    //
    // `cast_possible_truncation` + `cast_sign_loss` allowed: tet/cell
    // counts and floor/ceil indices are bounded by world extent /
    // cell_size, well below `u32::MAX` (γ-locked TetId invariant).
    // `cast_precision_loss` allowed: index → f64 in cell-size pick.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    pub(super) fn build<M: Material>(mesh: &dyn Mesh<M>, positions: &[Vec3]) -> Self {
        let n_tets = mesh.n_tets();

        // Mesh AABB.
        let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in positions {
            min = min.inf(p);
            max = max.sup(p);
        }
        // Slight pad so probes exactly on the AABB boundary still hit a
        // valid cell after `floor`. Smaller than any meaningful tet edge.
        let pad = 1e-9_f64;
        min -= Vec3::new(pad, pad, pad);
        max += Vec3::new(pad, pad, pad);

        // Cell size: empirically tuned at `avg_tet_edge × 0.25` (see
        // the explicit comment + measurement just below the avg_edge
        // computation). Sample one edge per tet for the average.
        let mut total_edge = 0.0_f64;
        for t in 0..n_tets as u32 {
            let [v0, v1, _v2, _v3] = mesh.tet_vertices(t);
            total_edge += (positions[v1 as usize] - positions[v0 as usize]).norm();
        }
        let avg_edge = if n_tets > 0 {
            total_edge / n_tets as f64
        } else {
            1.0
        };
        // Empirically tuned on row 20 (51 k tets, ~50 k display vertices
        // per cardinal cross-section). `avg_edge × 0.25` minimises total
        // build + lookup time on uniform BCC + IS meshes; coarser cells
        // overload per-cell tet lists, finer cells overload the build.
        let cell_size = (avg_edge * 0.25).max(1e-6);

        let extent = max - min;
        let nx = ((extent.x / cell_size).ceil() as usize).max(1);
        let ny = ((extent.y / cell_size).ceil() as usize).max(1);
        let nz = ((extent.z / cell_size).ceil() as usize).max(1);
        let mut cells: Vec<Vec<u32>> = vec![Vec::new(); nx * ny * nz];

        for t in 0..n_tets as u32 {
            let [v0, v1, v2, v3] = mesh.tet_vertices(t);
            let p0 = positions[v0 as usize];
            let p1 = positions[v1 as usize];
            let p2 = positions[v2 as usize];
            let p3 = positions[v3 as usize];
            let tet_min = p0.inf(&p1).inf(&p2).inf(&p3);
            let tet_max = p0.sup(&p1).sup(&p2).sup(&p3);

            let ix0 = (((tet_min.x - min.x) / cell_size).floor() as usize).min(nx - 1);
            let iy0 = (((tet_min.y - min.y) / cell_size).floor() as usize).min(ny - 1);
            let iz0 = (((tet_min.z - min.z) / cell_size).floor() as usize).min(nz - 1);
            let ix1 = (((tet_max.x - min.x) / cell_size).ceil() as usize)
                .min(nx)
                .saturating_sub(1)
                .max(ix0);
            let iy1 = (((tet_max.y - min.y) / cell_size).ceil() as usize)
                .min(ny)
                .saturating_sub(1)
                .max(iy0);
            let iz1 = (((tet_max.z - min.z) / cell_size).ceil() as usize)
                .min(nz)
                .saturating_sub(1)
                .max(iz0);

            for iz in iz0..=iz1 {
                for iy in iy0..=iy1 {
                    for ix in ix0..=ix1 {
                        cells[iz * nx * ny + iy * nx + ix].push(t);
                    }
                }
            }
        }

        Self {
            origin: min,
            cell_size,
            nx,
            ny,
            nz,
            cells,
        }
    }

    /// Append tet IDs registered in `probe`'s grid cell plus the 26
    /// neighbors (3×3×3 window) into `out` (caller clears and reuses
    /// the buffer to avoid per-probe allocations). The neighbor
    /// expansion catches tets whose AABB the probe is just outside —
    /// needed for the nearest-tet fallback in
    /// [`sample_analysis_at_point`] when the design SDF surface
    /// doesn't coincide with the analysis mesh's polyhedral envelope.
    /// Duplicates are not removed; per-probe linear walk tolerates them
    /// (`sample_analysis_at_point` is idempotent under repeated tets).
    //
    // `cast_possible_truncation` + `cast_sign_loss` + `cast_possible_wrap`
    // allowed: grid dimensions are bounded by world extent / cell_size
    // which fits well below i64::MAX in any practical deployment; the
    // i64 work only exists to make the dx/dy/dz=-1..=1 boundary check
    // expressible without underflow on the lower-corner cell.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_possible_wrap
    )]
    pub(super) fn append_candidates_with_neighbors(&self, probe: Vec3, out: &mut Vec<u32>) {
        let ix = ((probe.x - self.origin.x) / self.cell_size).floor() as i64;
        let iy = ((probe.y - self.origin.y) / self.cell_size).floor() as i64;
        let iz = ((probe.z - self.origin.z) / self.cell_size).floor() as i64;

        for dz in -1_i64..=1 {
            let nz = iz + dz;
            if nz < 0 || nz >= self.nz as i64 {
                continue;
            }
            for dy in -1_i64..=1 {
                let ny = iy + dy;
                if ny < 0 || ny >= self.ny as i64 {
                    continue;
                }
                for dx in -1_i64..=1 {
                    let nx = ix + dx;
                    if nx < 0 || nx >= self.nx as i64 {
                        continue;
                    }
                    let key = nz as usize * self.nx * self.ny + ny as usize * self.nx + nx as usize;
                    out.extend_from_slice(&self.cells[key]);
                }
            }
        }
    }
}

/// Find the analysis-mesh tet enclosing `probe` and return the
/// barycentric-interpolated per-vertex displacement; same enclosing-
/// tet-or-nearest-fallback logic as [`sample_analysis_at_point`] but
/// returning a [`Vec3`] instead of a per-scalar [`Vec`]. Used by
/// [`design_surface_deformed`].
//
// `cast_possible_truncation` allowed for VertexId casts (u32 crate-wide).
#[allow(clippy::cast_possible_truncation, clippy::similar_names)]
pub(super) fn sample_displacement_at_point<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
    displacement_per_vertex: &[Vec3],
) -> Vec3 {
    let tol_bary = -1e-9_f64;
    let mut best_outside_t: Option<u32> = None;
    let mut best_outside_min_b = f64::NEG_INFINITY;

    for &t in candidate_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        if m.determinant().abs() < 1e-30 {
            continue;
        }
        let Some(inv) = m.try_inverse() else {
            continue;
        };
        let b = inv * (probe - p0);
        let b0 = 1.0 - b.x - b.y - b.z;
        let bs = [b0, b.x, b.y, b.z];
        let min_b = bs.iter().copied().fold(f64::INFINITY, f64::min);
        if min_b >= tol_bary {
            return bs[0] * displacement_per_vertex[v0 as usize]
                + bs[1] * displacement_per_vertex[v1 as usize]
                + bs[2] * displacement_per_vertex[v2 as usize]
                + bs[3] * displacement_per_vertex[v3 as usize];
        }
        if min_b > best_outside_min_b {
            best_outside_min_b = min_b;
            best_outside_t = Some(t);
        }
    }

    if let Some(t) = best_outside_t {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let p0 = positions[v0 as usize];
        let p1 = positions[v1 as usize];
        let p2 = positions[v2 as usize];
        let p3 = positions[v3 as usize];
        let m = Matrix3::from_columns(&[p1 - p0, p2 - p0, p3 - p0]);
        if let Some(inv) = m.try_inverse() {
            let b = inv * (probe - p0);
            let b0 = 1.0 - b.x - b.y - b.z;
            let mut bs = [b0.max(0.0), b.x.max(0.0), b.y.max(0.0), b.z.max(0.0)];
            let sum: f64 = bs.iter().sum();
            if sum > 0.0 {
                for v in &mut bs {
                    *v /= sum;
                }
                return bs[0] * displacement_per_vertex[v0 as usize]
                    + bs[1] * displacement_per_vertex[v1 as usize]
                    + bs[2] * displacement_per_vertex[v2 as usize]
                    + bs[3] * displacement_per_vertex[v3 as usize];
            }
        }
    }
    Vec3::zeros()
}

/// C2 categorical-scalar predicate: a per-tet scalar whose **name** ends
/// in `_id` is treated as a categorical integer label (e.g.
/// `material_shell_id`, `material_zone_id`, `primitive_id`) and routed
/// through the nearest-tet path that preserves the integer; anything
/// else (`psi_j_per_m3`, `displacement_magnitude`, …) is treated as a
/// continuous field and routed through the per-primitive continuous-
/// scalar path — volume-weighted-per-vertex (on [`boundary_surface`]),
/// linear-along-edge from the volume-weighted-per-vertex field (on
/// [`slab_cut`] / [`slab_cut_deformed`] / [`design_slab_cut`]), or
/// Shepard kNN-IDW (on [`design_surface`] and its deformed/scene
/// siblings; see [`idw_k_nearest_tet_centroids`]).
///
/// Continuous-scalar smoothing breaks the categorical invariant —
/// averaging a `_id = 1` tet with a `_id = 2` tet yields `1.5`, which
/// the cf-view colormap detector reads as Sequential rather than
/// Categorical (it falls back to viridis at every layer boundary).
/// Routing categorical scalars to nearest-tet (k=1) keeps display-vertex
/// samples exactly integer, preserving the tab10 categorical render.
///
/// Suffix-by-name is the lightest convention that matches the existing
/// producer code (`material_shell_id`, `material_zone_id`,
/// `primitive_id`); per-scalar metadata on [`mesh_types::AttributedMesh`]
/// is a cleaner long-term answer if `_id` ever becomes a footgun, and is
/// banked as a C2 followup.
#[inline]
#[must_use]
pub(super) fn is_categorical_scalar_name(name: &str) -> bool {
    name.ends_with("_id")
}

/// Pick the tet from `candidate_tets` whose centroid is closest to
/// `probe`, returning its tet ID. Returns `None` when `candidate_tets`
/// is empty.
///
/// Used by the C2 categorical sampling path on [`design_surface`]
/// (and its deformed/scene siblings) and on [`design_slab_cut`]: one
/// nearest-tet lookup per probe is shared across every categorical
/// scalar in flight (all categorical scalars sample from the same
/// nearest tet, so the centroid search amortizes). [`slab_cut`] /
/// [`slab_cut_deformed`] use a related position-based pick via
/// `SlabCutState`'s `cut_owner_dist2` tracking rather than calling
/// this helper directly — the cut-vertex's edge-star tets are visited
/// one-at-a-time as the marching-tet loop iterates, so the winner
/// promotion happens incrementally on edge dedup.
//
// `cast_possible_truncation` allowed: VertexId/TetId are u32 by crate
// convention so `mesh.n_tets() < u32::MAX` is a γ-locked invariant.
#[allow(clippy::cast_possible_truncation)]
pub(super) fn nearest_tet_centroid_idx<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
) -> Option<u32> {
    let mut best: Option<u32> = None;
    let mut best_d2 = f64::INFINITY;
    for &t in candidate_tets {
        let [v0, v1, v2, v3] = mesh.tet_vertices(t);
        let centroid = (positions[v0 as usize]
            + positions[v1 as usize]
            + positions[v2 as usize]
            + positions[v3 as usize])
            * 0.25;
        let d2 = (probe - centroid).norm_squared();
        if d2 < best_d2 {
            best_d2 = d2;
            best = Some(t);
        }
    }
    best
}

/// C2.2 continuous-scalar sampler: Shepard-style k-nearest-tet IDW
/// (inverse-distance-weighted average) at an arbitrary 3D probe point.
///
/// Returns one f64 per scalar in `per_tet_scalars` order. Picks the
/// `k` tets from `candidate_tets` with smallest centroid-to-probe
/// distance and computes the weighted average:
/// `value = Σ (w_i * per_tet[t_i]) / Σ w_i` where
/// `w_i = 1 / (d_i² + eps)` and `d_i` is the centroid-to-probe
/// distance for tet `t_i`. The `eps` floor avoids weight blow-up when
/// the probe coincides with a centroid (the limit collapses to
/// nearest-tet behavior, which matches the C2.1 categorical path).
///
/// Wider smoothing radius than the pre-C2.2 vol-weighted-per-vertex +
/// barycentric path: that path averaged over each MC vertex's one
/// enclosing analysis tet, producing piecewise-linear-per-tet output
/// with gradient jumps at tet faces (visible as a "patchwork" pattern
/// on the ψ field of `design_surface` output). kNN-IDW averages over
/// k tets regardless of enclosing-tet boundaries, producing visibly
/// smoother gradients on the design-mesh display surface.
///
/// `candidate_tets` is expected to be a `TetGrid` 3×3×3 cell window
/// around the probe; empty candidates → returns `vec![0.0; n_scalars]`
/// (orphan-probe convention shared with the rest of the module).
//
// `cast_possible_truncation` allowed: VertexId/TetId are u32 by crate
// convention; `mesh.n_tets() < u32::MAX` is a γ-locked invariant.
#[allow(clippy::cast_possible_truncation)]
pub(super) fn idw_k_nearest_tet_centroids<M: Material>(
    probe: Vec3,
    mesh: &dyn Mesh<M>,
    positions: &[Vec3],
    candidate_tets: &[u32],
    per_tet_scalars: &[&[f64]],
    k: usize,
    eps: f64,
) -> Vec<f64> {
    let n_scalars = per_tet_scalars.len();
    if candidate_tets.is_empty() || n_scalars == 0 {
        return vec![0.0; n_scalars];
    }

    // (squared_distance_to_centroid, tet_id) for each candidate.
    let mut by_dist2: Vec<(f64, u32)> = candidate_tets
        .iter()
        .map(|&t| {
            let [v0, v1, v2, v3] = mesh.tet_vertices(t);
            let centroid = (positions[v0 as usize]
                + positions[v1 as usize]
                + positions[v2 as usize]
                + positions[v3 as usize])
                * 0.25;
            ((probe - centroid).norm_squared(), t)
        })
        .collect();

    // Sort ascending by squared distance. For BCC + 3×3×3 TetGrid
    // window the candidate count is ~150 and k is ~8; full sort is
    // cheap and simpler than a partial-sort heap.
    by_dist2.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let k_eff = k.min(by_dist2.len());

    let mut total_weight = 0.0_f64;
    let mut sums = vec![0.0_f64; n_scalars];
    for &(d2, t) in &by_dist2[..k_eff] {
        let w = 1.0 / (d2 + eps);
        total_weight += w;
        for (i, &per_tet) in per_tet_scalars.iter().enumerate() {
            sums[i] = w.mul_add(per_tet[t as usize], sums[i]);
        }
    }

    if total_weight > 0.0 {
        sums.iter().map(|&s| s / total_weight).collect()
    } else {
        // All weights numerically zero — logically unreachable: the
        // empty-candidates case returns at the top of the fn, and
        // `1 / (d² + eps)` is strictly positive for any finite `d²`
        // with `eps > 0`. Keep the branch as a defensive zero-fill
        // so the caller's per-scalar Vec stays correctly sized.
        vec![0.0; n_scalars]
    }
}

/// Volume-weighted projection of a per-tet scalar to per-vertex.
///
/// `per_tet_scalar.len()` must equal `mesh.n_tets()`; callers are
/// expected to validate this first via [`validate_per_tet_scalars`]
/// (the public viz fns do).
//
// `cast_possible_truncation` allowed: TetId is u32 by crate
// convention so `mesh.n_tets() < u32::MAX` is a γ-locked invariant.
#[allow(clippy::cast_possible_truncation)]
pub(super) fn volume_weighted_per_vertex_avg<M: Material>(
    mesh: &dyn Mesh<M>,
    per_tet_scalar: &[f64],
) -> Vec<f64> {
    let n_vertices = mesh.n_vertices();
    let signed_volumes = &mesh.quality().signed_volume;
    let mut accum_val_vol = vec![0.0_f64; n_vertices];
    let mut accum_vol = vec![0.0_f64; n_vertices];
    for t in 0..mesh.n_tets() {
        // signed_volume comes out positive for right-handed tets per
        // pipeline Decision H; .abs() is defensive.
        let vol = signed_volumes[t].abs();
        let weighted = per_tet_scalar[t] * vol;
        let [v0, v1, v2, v3] = mesh.tet_vertices(t as u32);
        for v in [v0, v1, v2, v3] {
            accum_val_vol[v as usize] += weighted;
            accum_vol[v as usize] += vol;
        }
    }
    (0..n_vertices)
        .map(|v| {
            if accum_vol[v] > 0.0 {
                accum_val_vol[v] / accum_vol[v]
            } else {
                // Orphan vertex (no incident tets) — shouldn't occur
                // on connected meshes; return 0 so the scalar slot
                // stays valid.
                0.0
            }
        })
        .collect()
}

pub(super) fn validate_per_tet_scalars<M: Material>(
    mesh: &dyn Mesh<M>,
    per_tet_scalars: &BTreeMap<&str, &[f64]>,
) -> Result<(), VizError> {
    let expected = mesh.n_tets();
    for (&name, &values) in per_tet_scalars {
        if values.len() != expected {
            return Err(VizError::PerTetScalarLengthMismatch {
                name: name.to_string(),
                expected,
                actual: values.len(),
            });
        }
    }
    Ok(())
}
