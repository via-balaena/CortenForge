//! Fitting pipeline implementation.

use crate::{
    ControlRegion, FitParams, FitResult, FitStage, FitTemplate, MeasurementType, RegionDefinition,
    TemplateError, TemplateResult,
};
use mesh_morph::{Constraint, MorphParams};
use mesh_registration::{IcpParams, RigidTransform, icp_align, transform_mesh};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

/// Fits a template to the given parameters.
///
/// Implements the three-stage fitting pipeline:
/// 1. Rigid alignment (ICP) if target scan is provided
/// 2. Landmark deformation (RBF morphing) if landmark targets are provided
/// 3. Measurement adjustment if measurement targets are provided
pub fn fit_template(template: &FitTemplate, params: &FitParams) -> TemplateResult<FitResult> {
    // Validate inputs
    if template.mesh.vertices.is_empty() {
        return Err(TemplateError::EmptyMesh);
    }

    if !params.has_constraints() {
        return Err(TemplateError::NoConstraints);
    }

    let mut mesh = template.mesh.clone();
    let mut stages = Vec::new();
    let mut cumulative_transform = RigidTransform::identity();

    // Stage 1: Rigid alignment
    if let Some(scan) = &params.target_scan {
        if scan.vertices.is_empty() {
            return Err(TemplateError::EmptyScan);
        }

        #[allow(clippy::cast_possible_truncation)]
        let icp_params = IcpParams::new()
            .with_max_iterations(params.registration_iterations as u32)
            .with_convergence_threshold(params.convergence_threshold);

        let icp_result = icp_align(&mesh, scan, &icp_params)?;

        mesh = transform_mesh(&mesh, &icp_result.transform);
        cumulative_transform = icp_result.transform;

        stages.push(FitStage::RigidAlignment {
            rms_error: icp_result.rms_error,
            iterations: icp_result.iterations as usize,
            converged: icp_result.converged,
        });
    }

    // Stage 2: Landmark deformation
    if !params.landmark_targets.is_empty() {
        let (deformed_mesh, stage) = apply_landmark_deformation(&mesh, template, params)?;
        mesh = deformed_mesh;
        stages.push(stage);
    }

    // Stage 3: Measurement adjustment
    if !params.measurement_targets.is_empty() {
        let (adjusted_mesh, stage) = apply_measurement_adjustment(&mesh, template, params)?;
        mesh = adjusted_mesh;
        stages.push(stage);
    }

    // Calculate final fit error
    let fit_error = calculate_fit_error(&mesh, template, params);

    Ok(FitResult {
        mesh,
        fit_error,
        stages,
        transform: cumulative_transform,
    })
}

/// Applies landmark-based deformation using RBF morphing.
fn apply_landmark_deformation(
    mesh: &IndexedMesh,
    template: &FitTemplate,
    params: &FitParams,
) -> TemplateResult<(IndexedMesh, FitStage)> {
    let mut constraints = Vec::new();
    let mut max_displacement = 0.0;

    for (name, target) in &params.landmark_targets {
        // Get the current position of this landmark
        let region = template
            .get_region(name)
            .ok_or_else(|| TemplateError::RegionNotFound { name: name.clone() })?;

        // Skip preserved regions
        if region.preserve {
            continue;
        }

        // Get source position (from mesh, accounting for any previous transforms)
        let source_pos = get_region_centroid(region, mesh);

        // Create constraint
        let constraint = Constraint::weighted(source_pos, *target, region.weight);
        constraints.push(constraint);

        // Track displacement
        let displacement = (target - source_pos).norm();
        if displacement > max_displacement {
            max_displacement = displacement;
        }
    }

    // Add constraints for preserved regions to keep them in place
    for region in template.control_regions().values() {
        if region.preserve {
            let pos = get_region_centroid(region, mesh);
            constraints.push(Constraint::point(pos, pos));
        }
    }

    let constraints_applied = constraints.len();

    if constraints.is_empty() {
        // No constraints to apply, return mesh unchanged
        return Ok((
            mesh.clone(),
            FitStage::LandmarkDeformation {
                constraints_applied: 0,
                max_displacement: 0.0,
            },
        ));
    }

    // Apply RBF morphing
    let morph_params = MorphParams::rbf().with_constraints(constraints);

    let morph_result = mesh_morph::morph_mesh(mesh, &morph_params)?;

    Ok((
        morph_result.mesh,
        FitStage::LandmarkDeformation {
            constraints_applied,
            max_displacement,
        },
    ))
}

/// Applies measurement-based adjustment by scaling regions.
fn apply_measurement_adjustment(
    mesh: &IndexedMesh,
    template: &FitTemplate,
    params: &FitParams,
) -> TemplateResult<(IndexedMesh, FitStage)> {
    let mut current_mesh = mesh.clone();
    let mut measurements_applied = 0;
    let mut max_scale_factor: f64 = 1.0;

    for (name, target_measurement) in &params.measurement_targets {
        // Get the measurement region
        let region = template
            .get_region(name)
            .ok_or_else(|| TemplateError::RegionNotFound { name: name.clone() })?;

        // Extract measurement type and plane from region
        let (measurement_type, origin, normal) = match &region.definition {
            RegionDefinition::MeasurementPlane {
                measurement_type,
                origin,
                normal,
            } => (*measurement_type, *origin, *normal),
            _ => {
                // Non-measurement region, skip
                continue;
            }
        };

        // Get vertices in the measurement region
        let vertex_indices = region.get_vertex_indices(&current_mesh);
        if vertex_indices.is_empty() {
            continue;
        }

        // Compute current measurement
        let current_value = compute_measurement(
            &current_mesh,
            &vertex_indices,
            measurement_type,
            origin,
            normal,
        );

        // Compute scale factor
        if let Some(scale) = target_measurement.scale_factor(current_value) {
            if (scale - 1.0).abs() > 1e-6 {
                // Apply scaling to the mesh
                current_mesh = apply_measurement_scale(
                    &current_mesh,
                    &vertex_indices,
                    scale,
                    measurement_type,
                    origin,
                    normal,
                );

                if scale.abs() > max_scale_factor.abs() {
                    max_scale_factor = scale;
                }
                measurements_applied += 1;
            }
        }
    }

    Ok((
        current_mesh,
        FitStage::MeasurementAdjustment {
            measurements_applied,
            max_scale_factor,
        },
    ))
}

/// Gets the centroid of a region in the current mesh state.
fn get_region_centroid(region: &ControlRegion, mesh: &IndexedMesh) -> Point3<f64> {
    match &region.definition {
        RegionDefinition::Point(pos) => {
            // Find closest vertex in mesh
            let mut closest = *pos;
            let mut min_dist = f64::MAX;
            for v in &mesh.vertices {
                let dist = (v.position - pos).norm();
                if dist < min_dist {
                    min_dist = dist;
                    closest = v.position;
                }
            }
            closest
        }
        _ => region
            .centroid_in_mesh(mesh)
            .unwrap_or_else(|| region.centroid()),
    }
}

/// Computes a measurement value from mesh vertices.
fn compute_measurement(
    mesh: &IndexedMesh,
    vertex_indices: &std::collections::HashSet<u32>,
    measurement_type: MeasurementType,
    origin: Point3<f64>,
    normal: Vector3<f64>,
) -> f64 {
    if vertex_indices.is_empty() {
        return 0.0;
    }

    // Collect vertices
    let vertices: Vec<Point3<f64>> = vertex_indices
        .iter()
        .filter_map(|&idx| mesh.vertices.get(idx as usize).map(|v| v.position))
        .collect();

    if vertices.is_empty() {
        return 0.0;
    }

    match measurement_type {
        MeasurementType::Circumference => {
            // Project vertices onto plane and compute approximate circumference
            compute_circumference(&vertices, origin, normal)
        }
        MeasurementType::Width => {
            // Compute extent perpendicular to normal (first perpendicular axis)
            compute_extent(&vertices, normal, true)
        }
        MeasurementType::Height => {
            // Compute extent along normal
            compute_extent_along_normal(&vertices, normal)
        }
        MeasurementType::Depth => {
            // Compute extent perpendicular to normal (second perpendicular axis)
            compute_extent(&vertices, normal, false)
        }
    }
}

/// Computes circumference by projecting points onto a plane and computing perimeter.
fn compute_circumference(
    vertices: &[Point3<f64>],
    origin: Point3<f64>,
    normal: Vector3<f64>,
) -> f64 {
    if vertices.len() < 3 {
        return 0.0;
    }

    // Create local coordinate system on the plane
    let (u_axis, v_axis) = create_orthonormal_basis(normal);

    // Project points onto plane
    let projected: Vec<(f64, f64)> = vertices
        .iter()
        .map(|p| {
            let to_point = p - origin;
            let u = to_point.dot(&u_axis);
            let v = to_point.dot(&v_axis);
            (u, v)
        })
        .collect();

    // Compute centroid of projected points
    let mut cx = 0.0;
    let mut cy = 0.0;
    for (u, v) in &projected {
        cx += u;
        cy += v;
    }
    #[allow(clippy::cast_precision_loss)]
    {
        cx /= projected.len() as f64;
        cy /= projected.len() as f64;
    }

    // Compute average radius
    let mut total_radius = 0.0;
    for (u, v) in &projected {
        let dx = u - cx;
        let dy = v - cy;
        total_radius += dx.hypot(dy);
    }
    #[allow(clippy::cast_precision_loss)]
    let avg_radius = total_radius / projected.len() as f64;

    // Circumference = 2 * pi * r
    2.0 * std::f64::consts::PI * avg_radius
}

/// Computes extent along an axis perpendicular to normal.
fn compute_extent(vertices: &[Point3<f64>], normal: Vector3<f64>, first_axis: bool) -> f64 {
    if vertices.is_empty() {
        return 0.0;
    }

    let (u_axis, v_axis) = create_orthonormal_basis(normal);
    let axis = if first_axis { u_axis } else { v_axis };

    let mut min_proj = f64::MAX;
    let mut max_proj = f64::MIN;

    for v in vertices {
        let proj = v.coords.dot(&axis);
        min_proj = min_proj.min(proj);
        max_proj = max_proj.max(proj);
    }

    max_proj - min_proj
}

/// Computes extent along the normal direction.
fn compute_extent_along_normal(vertices: &[Point3<f64>], normal: Vector3<f64>) -> f64 {
    if vertices.is_empty() {
        return 0.0;
    }

    let mut min_proj = f64::MAX;
    let mut max_proj = f64::MIN;

    for v in vertices {
        let proj = v.coords.dot(&normal);
        min_proj = min_proj.min(proj);
        max_proj = max_proj.max(proj);
    }

    max_proj - min_proj
}

/// Creates an orthonormal basis from a normal vector.
fn create_orthonormal_basis(normal: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Find a vector not parallel to normal
    let arbitrary = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };

    let u = normal.cross(&arbitrary).normalize();
    let v = normal.cross(&u).normalize();

    (u, v)
}

/// Applies measurement-based scaling to mesh vertices.
fn apply_measurement_scale(
    mesh: &IndexedMesh,
    vertex_indices: &std::collections::HashSet<u32>,
    scale: f64,
    measurement_type: MeasurementType,
    origin: Point3<f64>,
    normal: Vector3<f64>,
) -> IndexedMesh {
    let mut result = mesh.clone();

    let (u_axis, v_axis) = create_orthonormal_basis(normal);

    for &idx in vertex_indices {
        if let Some(vertex) = result.vertices.get_mut(idx as usize) {
            let to_point = vertex.position - origin;

            // Decompose into components
            let along_normal = to_point.dot(&normal);
            let along_u = to_point.dot(&u_axis);
            let along_v = to_point.dot(&v_axis);

            // Apply scaling based on measurement type
            let new_pos = match measurement_type {
                MeasurementType::Circumference => {
                    // Scale perpendicular to normal (radially)
                    origin
                        + normal * along_normal
                        + u_axis * (along_u * scale)
                        + v_axis * (along_v * scale)
                }
                MeasurementType::Width => {
                    // Scale along u axis only
                    origin + normal * along_normal + u_axis * (along_u * scale) + v_axis * along_v
                }
                MeasurementType::Height => {
                    // Scale along normal
                    origin + normal * (along_normal * scale) + u_axis * along_u + v_axis * along_v
                }
                MeasurementType::Depth => {
                    // Scale along v axis only
                    origin + normal * along_normal + u_axis * along_u + v_axis * (along_v * scale)
                }
            };

            vertex.position = new_pos;
        }
    }

    result
}

/// Calculates the overall fit error.
fn calculate_fit_error(mesh: &IndexedMesh, template: &FitTemplate, params: &FitParams) -> f64 {
    let mut total_error = 0.0;
    let mut total_weight = 0.0;

    // Error from landmark targets
    for (name, target) in &params.landmark_targets {
        if let Some(region) = template.get_region(name) {
            let current_pos = get_region_centroid(region, mesh);
            let error = (current_pos - target).norm();
            total_error += error * error * region.weight;
            total_weight += region.weight;
        }
    }

    if total_weight > 0.0 {
        (total_error / total_weight).sqrt()
    } else {
        0.0
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn make_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 1.0));

        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh.faces.push([4, 6, 5]);
        mesh.faces.push([4, 7, 6]);
        mesh.faces.push([0, 4, 5]);
        mesh.faces.push([0, 5, 1]);
        mesh.faces.push([2, 6, 7]);
        mesh.faces.push([2, 7, 3]);
        mesh.faces.push([0, 7, 4]);
        mesh.faces.push([0, 3, 7]);
        mesh.faces.push([1, 5, 6]);
        mesh.faces.push([1, 6, 2]);

        mesh
    }

    #[test]
    fn test_create_orthonormal_basis() {
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let (u, v) = create_orthonormal_basis(normal);

        // Check orthogonality
        assert_relative_eq!(u.dot(&v), 0.0, epsilon = 1e-10);
        assert_relative_eq!(u.dot(&normal), 0.0, epsilon = 1e-10);
        assert_relative_eq!(v.dot(&normal), 0.0, epsilon = 1e-10);

        // Check unit vectors
        assert_relative_eq!(u.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(v.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_create_orthonormal_basis_x_aligned() {
        let normal = Vector3::new(1.0, 0.0, 0.0);
        let (u, v) = create_orthonormal_basis(normal);

        assert_relative_eq!(u.dot(&normal), 0.0, epsilon = 1e-10);
        assert_relative_eq!(v.dot(&normal), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_extent_along_normal() {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(0.0, 0.0, 10.0),
        ];
        let normal = Vector3::new(0.0, 0.0, 1.0);

        let extent = compute_extent_along_normal(&vertices, normal);
        assert_relative_eq!(extent, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compute_extent_perpendicular() {
        // Vertices along X axis
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ];
        let normal = Vector3::new(0.0, 0.0, 1.0);

        // For normal = Z, the orthonormal basis is:
        // u = Y axis (first_axis = true)
        // v = -X axis (first_axis = false)
        // Since vertices are along X, use second axis
        let extent = compute_extent(&vertices, normal, false);
        assert!(extent > 9.0, "Extent should be ~10: {extent}");
    }

    #[test]
    fn test_fit_empty_mesh() {
        let mesh = IndexedMesh::new();
        let template = FitTemplate::new(mesh);
        let params = FitParams::new().with_landmark_target("test", Point3::origin());

        let result = fit_template(&template, &params);
        assert!(matches!(result, Err(TemplateError::EmptyMesh)));
    }

    #[test]
    fn test_fit_no_constraints() {
        let mesh = make_cube();
        let template = FitTemplate::new(mesh);

        let result = fit_template(&template, &FitParams::new());
        assert!(matches!(result, Err(TemplateError::NoConstraints)));
    }

    #[test]
    fn test_fit_missing_region() {
        let mesh = make_cube();
        let template = FitTemplate::new(mesh);
        let params = FitParams::new().with_landmark_target("missing", Point3::origin());

        let result = fit_template(&template, &params);
        assert!(matches!(result, Err(TemplateError::RegionNotFound { .. })));
    }

    #[test]
    fn test_fit_with_landmark() {
        let mesh = make_cube();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("corner", Point3::new(0.0, 0.0, 0.0)));

        let params = FitParams::new().with_landmark_target("corner", Point3::new(0.0, 0.0, 0.0));

        let result = fit_template(&template, &params).unwrap();
        assert!(result.has_landmark_deformation());
    }

    #[test]
    fn test_fit_with_scan() {
        let mesh = make_cube();
        let scan = make_cube();

        let template = FitTemplate::new(mesh);
        let params = FitParams::new().with_target_scan(scan);

        let result = fit_template(&template, &params).unwrap();
        assert!(result.has_rigid_alignment());
    }

    #[test]
    fn test_fit_with_empty_scan() {
        let mesh = make_cube();
        let template = FitTemplate::new(mesh);
        let params = FitParams::new().with_target_scan(IndexedMesh::new());

        let result = fit_template(&template, &params);
        assert!(matches!(result, Err(TemplateError::EmptyScan)));
    }

    #[test]
    fn test_calculate_fit_error_no_targets() {
        let mesh = make_cube();
        let template = FitTemplate::new(mesh.clone());
        let params = FitParams::new();

        let error = calculate_fit_error(&mesh, &template, &params);
        assert_relative_eq!(error, 0.0);
    }

    #[test]
    fn test_calculate_fit_error_with_targets() {
        let mesh = make_cube();
        let template = FitTemplate::new(mesh.clone())
            .with_control_region(ControlRegion::point("corner", Point3::new(0.0, 0.0, 0.0)));

        let params = FitParams::new().with_landmark_target("corner", Point3::new(1.0, 0.0, 0.0));

        let error = calculate_fit_error(&mesh, &template, &params);
        // Error should be ~1.0 (distance from (0,0,0) to (1,0,0))
        assert!(error > 0.9, "Error should be ~1.0: {error}");
    }

    #[test]
    fn test_apply_measurement_scale() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));

        let indices: std::collections::HashSet<u32> = [0, 1].into_iter().collect();

        let scaled = apply_measurement_scale(
            &mesh,
            &indices,
            2.0,
            MeasurementType::Circumference,
            Point3::origin(),
            Vector3::z(),
        );

        // Vertices should be scaled radially by 2x
        assert!(scaled.vertices[0].position.x > 1.5);
        assert!(scaled.vertices[1].position.y > 1.5);
    }

    #[test]
    fn test_get_region_centroid_point() {
        let mesh = make_cube();
        let region = ControlRegion::point("test", Point3::new(0.0, 0.0, 0.0));

        let centroid = get_region_centroid(&region, &mesh);
        // Should find closest vertex at (0,0,0)
        assert_relative_eq!(centroid.x, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_get_region_centroid_vertices() {
        let mesh = make_cube();
        let region = ControlRegion::vertices("test", vec![0, 6]); // (0,0,0) and (1,1,1)

        let centroid = get_region_centroid(&region, &mesh);
        assert_relative_eq!(centroid.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(centroid.y, 0.5, epsilon = 1e-10);
        assert_relative_eq!(centroid.z, 0.5, epsilon = 1e-10);
    }
}
