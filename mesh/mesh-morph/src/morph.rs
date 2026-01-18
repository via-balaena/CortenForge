//! Main morphing function.
//!
//! This module provides the [`morph_mesh`] function which applies deformation
//! to a mesh based on the specified parameters.

use crate::{
    ffd::FfdLattice,
    params::MorphAlgorithm,
    rbf::RbfInterpolator,
    result::{compute_edge_stats, compute_signed_volume},
    MorphError, MorphOutput, MorphParams, MorphResult,
};
use mesh_types::{IndexedMesh, MeshBounds};
use rayon::prelude::*;

/// Morphs a mesh according to the specified parameters.
///
/// This function applies deformation to the mesh vertices based on the
/// constraints and algorithm specified in the parameters.
///
/// # Arguments
///
/// * `mesh` - The mesh to deform
/// * `params` - The morphing parameters
///
/// # Returns
///
/// A [`MorphOutput`] containing the deformed mesh and quality metrics.
///
/// # Errors
///
/// Returns an error if:
/// - The mesh is empty
/// - No constraints are provided
/// - The RBF system is degenerate
///
/// # Examples
///
/// ## Basic RBF Morphing
///
/// ```
/// use mesh_morph::{morph_mesh, MorphParams, Constraint};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::Point3;
///
/// // Create a simple triangle mesh
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// // Define constraints - move one vertex up
/// let params = MorphParams::rbf()
///     .with_constraint(Constraint::point(
///         Point3::new(0.0, 0.0, 0.0),
///         Point3::new(0.0, 0.0, 0.5),
///     ))
///     .with_constraint(Constraint::point(
///         Point3::new(1.0, 0.0, 0.0),
///         Point3::new(1.0, 0.0, 0.0),
///     ))
///     .with_constraint(Constraint::point(
///         Point3::new(0.0, 1.0, 0.0),
///         Point3::new(0.0, 1.0, 0.0),
///     ));
///
/// let result = morph_mesh(&mesh, &params).unwrap();
///
/// // First vertex should be moved up
/// assert!((result.mesh.vertices[0].position.z - 0.5).abs() < 0.1);
/// ```
///
/// ## FFD Morphing
///
/// ```
/// use mesh_morph::{morph_mesh, MorphParams, Constraint};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let params = MorphParams::ffd()
///     .with_constraint(Constraint::displacement(
///         Point3::new(0.5, 0.25, 0.0),
///         Vector3::new(0.0, 0.0, 0.2),
///     ));
///
/// let result = morph_mesh(&mesh, &params).unwrap();
/// assert!(result.vertices_modified > 0);
/// ```
pub fn morph_mesh(mesh: &IndexedMesh, params: &MorphParams) -> MorphResult<MorphOutput> {
    // Validate inputs
    if mesh.vertices.is_empty() {
        return Err(MorphError::EmptyMesh);
    }

    if params.constraints.is_empty() {
        return Err(MorphError::NoConstraints);
    }

    // Compute original metrics
    let original_volume = compute_signed_volume(mesh).abs();
    let (orig_min_edge, orig_max_edge, _, _) = compute_edge_stats(mesh);

    // Apply the appropriate morphing algorithm
    let deformed = match &params.algorithm {
        MorphAlgorithm::Rbf(kernel) => apply_rbf_morph(mesh, params, *kernel)?,
        MorphAlgorithm::Ffd(config) => apply_ffd_morph(mesh, params, *config),
    };

    // Compute result metrics
    let mut vertices_modified = 0;
    let mut max_displacement = 0.0;
    let mut total_displacement = 0.0;

    for (i, (orig, deformed_v)) in mesh.vertices.iter().zip(deformed.vertices.iter()).enumerate() {
        let disp = (deformed_v.position - orig.position).norm();
        if disp > 1e-10 && params.should_deform_vertex(i) {
            vertices_modified += 1;
            if disp > max_displacement {
                max_displacement = disp;
            }
            total_displacement += disp;
        }
    }

    #[allow(clippy::cast_precision_loss)]
    let average_displacement = if vertices_modified > 0 {
        total_displacement / vertices_modified as f64
    } else {
        0.0
    };

    // Compute edge stretch/compression
    let (new_min_edge, new_max_edge, _, _) = compute_edge_stats(&deformed);
    let max_stretch = if orig_max_edge > 1e-10 {
        new_max_edge / orig_max_edge
    } else {
        1.0
    };
    let max_compression = if orig_min_edge > 1e-10 && new_min_edge > 1e-10 {
        new_min_edge / orig_min_edge
    } else {
        1.0
    };

    // Compute volume ratio
    let new_volume = compute_signed_volume(&deformed).abs();
    let volume_ratio = if original_volume > 1e-10 {
        new_volume / original_volume
    } else {
        1.0
    };

    Ok(MorphOutput {
        mesh: deformed,
        vertices_modified,
        max_displacement,
        average_displacement,
        max_stretch,
        max_compression,
        volume_ratio,
    })
}

/// Applies RBF morphing to a mesh.
fn apply_rbf_morph(
    mesh: &IndexedMesh,
    params: &MorphParams,
    kernel: crate::RbfKernel,
) -> MorphResult<IndexedMesh> {
    let interpolator = RbfInterpolator::new(&params.constraints, kernel)?;

    let mut result = mesh.clone();

    // Determine which vertices to process
    let indices_to_process: Vec<usize> = (0..mesh.vertices.len())
        .filter(|i| params.should_deform_vertex(*i))
        .collect();

    // Process vertices in parallel for large meshes
    if indices_to_process.len() > 1000 {
        let new_positions: Vec<_> = indices_to_process
            .par_iter()
            .map(|&i| {
                let pos = &mesh.vertices[i].position;
                (i, interpolator.transform(pos))
            })
            .collect();

        for (i, new_pos) in new_positions {
            result.vertices[i].position = new_pos;
        }
    } else {
        for &i in &indices_to_process {
            let pos = &mesh.vertices[i].position;
            result.vertices[i].position = interpolator.transform(pos);
        }
    }

    Ok(result)
}

/// Applies FFD morphing to a mesh.
fn apply_ffd_morph(
    mesh: &IndexedMesh,
    params: &MorphParams,
    config: crate::FfdConfig,
) -> IndexedMesh {
    // Compute mesh bounds
    let bounds = mesh.bounds();

    // Create the FFD lattice
    let mut lattice = FfdLattice::new(&bounds, config);

    // Apply constraints to the lattice
    lattice.apply_constraints(&params.constraints);

    let mut result = mesh.clone();

    // Determine which vertices to process
    let indices_to_process: Vec<usize> = (0..mesh.vertices.len())
        .filter(|i| params.should_deform_vertex(*i))
        .collect();

    // Process vertices in parallel for large meshes
    if indices_to_process.len() > 1000 {
        let new_positions: Vec<_> = indices_to_process
            .par_iter()
            .map(|&i| {
                let pos = &mesh.vertices[i].position;
                (i, lattice.transform(pos))
            })
            .collect();

        for (i, new_pos) in new_positions {
            result.vertices[i].position = new_pos;
        }
    } else {
        for &i in &indices_to_process {
            let pos = &mesh.vertices[i].position;
            result.vertices[i].position = lattice.transform(pos);
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Constraint;
    use mesh_types::Vertex;
    use nalgebra::{Point3, Vector3};
    use std::collections::HashSet;

    fn make_test_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn make_test_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 1.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);
        mesh
    }

    #[test]
    fn test_empty_mesh_error() {
        let mesh = IndexedMesh::new();
        let params = MorphParams::rbf().with_constraint(Constraint::point(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
        ));

        let result = morph_mesh(&mesh, &params);
        assert!(matches!(result, Err(MorphError::EmptyMesh)));
    }

    #[test]
    fn test_no_constraints_error() {
        let mesh = make_test_triangle();
        let params = MorphParams::rbf();

        let result = morph_mesh(&mesh, &params);
        assert!(matches!(result, Err(MorphError::NoConstraints)));
    }

    #[test]
    fn test_rbf_identity_morph() {
        let mesh = make_test_triangle();

        // Identity constraints - points map to themselves
        let params = MorphParams::rbf()
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Vertices should stay in place (or very close)
        for (orig, deformed) in mesh.vertices.iter().zip(result.mesh.vertices.iter()) {
            let dist = (orig.position - deformed.position).norm();
            assert!(dist < 0.01, "Expected small displacement, got {}", dist);
        }
    }

    #[test]
    fn test_rbf_translation() {
        let mesh = make_test_triangle();
        let offset = Vector3::new(1.0, 2.0, 3.0);

        // Translate all vertices
        let params = MorphParams::rbf()
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0) + offset,
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0) + offset,
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0) + offset,
            ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Check all vertices are translated
        for (orig, deformed) in mesh.vertices.iter().zip(result.mesh.vertices.iter()) {
            let expected = orig.position + offset;
            let dist = (expected - deformed.position).norm();
            assert!(dist < 0.1, "Expected translation, error = {}", dist);
        }
    }

    #[test]
    fn test_ffd_morph() {
        let mesh = make_test_tetrahedron();

        let params = MorphParams::ffd().with_constraint(Constraint::displacement(
            Point3::new(0.5, 0.5, 0.5),
            Vector3::new(0.0, 0.0, 0.2),
        ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Some vertices should be modified
        assert!(result.vertices_modified > 0);
    }

    #[test]
    fn test_gaussian_kernel() {
        let mesh = make_test_triangle();

        let params = MorphParams::rbf_gaussian(1.0)
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.5),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();
        assert!(result.vertices_modified > 0);
    }

    #[test]
    fn test_multiquadric_kernel() {
        let mesh = make_test_triangle();

        let params = MorphParams::rbf_multiquadric(1.0)
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.5),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();
        assert!(result.vertices_modified > 0);
    }

    #[test]
    fn test_inverse_multiquadric_kernel() {
        let mesh = make_test_triangle();

        let params = MorphParams::rbf_inverse_multiquadric(1.0)
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.5),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();
        assert!(result.vertices_modified > 0);
    }

    #[test]
    fn test_vertex_mask() {
        let mesh = make_test_triangle();

        // Only deform vertex 0
        let mask: HashSet<usize> = [0].into_iter().collect();
        let params = MorphParams::rbf()
            .with_vertex_mask(mask)
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Vertex 0 should be moved
        let disp0 = (result.mesh.vertices[0].position - mesh.vertices[0].position).norm();
        assert!(disp0 > 0.1, "Vertex 0 should be moved");

        // Vertices 1 and 2 should stay in place
        let disp1 = (result.mesh.vertices[1].position - mesh.vertices[1].position).norm();
        let disp2 = (result.mesh.vertices[2].position - mesh.vertices[2].position).norm();
        assert!(disp1 < 1e-10, "Vertex 1 should not move");
        assert!(disp2 < 1e-10, "Vertex 2 should not move");
    }

    #[test]
    fn test_quality_metrics() {
        let mesh = make_test_tetrahedron();

        let params = MorphParams::rbf()
            .with_constraint(Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.5),
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.5, 1.0, 0.0),
                Point3::new(0.5, 1.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.5, 0.5, 1.0),
                Point3::new(0.5, 0.5, 1.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Check that metrics are computed
        assert!(result.max_displacement > 0.0);
        assert!(result.average_displacement > 0.0);
        assert!(result.max_stretch > 0.0);
        assert!(result.volume_ratio > 0.0);
    }

    #[test]
    fn test_weighted_constraints() {
        let mesh = make_test_triangle();

        // Strong weight on one constraint
        let params = MorphParams::rbf()
            .with_constraint(Constraint::weighted(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
                2.0,
            ))
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ))
            .with_constraint(Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ));

        let result = morph_mesh(&mesh, &params).unwrap();

        // Vertex 0 should move (even with weighted constraint)
        let disp = (result.mesh.vertices[0].position - mesh.vertices[0].position).norm();
        assert!(disp > 0.1, "Vertex 0 should be moved");
    }
}
