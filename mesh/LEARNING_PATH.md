# Mesh SDK Learning Path

A progressive, hands-on learning curriculum for the CortenForge Mesh SDK.
Complete these exercises in order to build mastery from foundational concepts to advanced operations.

## Prerequisites

- Rust programming knowledge (basics: structs, traits, Result types)
- A Rust development environment with `cargo`
- Clone and build the CortenForge repository

## How to Use This Guide

Each tier builds on the previous. For each exercise:

1. Read the objective and expected outcome
2. Write the code yourself (don't copy-paste)
3. Run your code and verify the output
4. Check off the exercise when complete
5. Reflect on what you learned before moving on

---

## Tier 1: Foundation

**Goal:** Understand core data types and mesh representation.

### Exercise 1.1: Create Your First Mesh

**Objective:** Create an `IndexedMesh` with a single triangle.

**Steps:**
```rust
use mesh::types::{IndexedMesh, Vertex};

fn main() {
    // Create an empty mesh
    let mut mesh = IndexedMesh::new();

    // Add three vertices forming a triangle
    mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));

    // Add one face connecting the vertices
    mesh.faces.push([0, 1, 2]);

    // Verify your mesh
    println!("Vertices: {}", mesh.vertex_count());
    println!("Faces: {}", mesh.face_count());
}
```

**Expected output:**
```
Vertices: 3
Faces: 1
```

**Reflection questions:**
- What is the difference between `vertex_count()` and `vertices.len()`?
- Why are faces stored as `[u32; 3]` arrays?

---

### Exercise 1.2: Use Primitive Generators

**Objective:** Generate a unit cube and explore its properties.

**Steps:**
```rust
use mesh::types::{unit_cube, MeshBounds};

fn main() {
    let cube = unit_cube();

    println!("Cube has {} vertices", cube.vertex_count());
    println!("Cube has {} faces", cube.face_count());

    // Get bounding box
    let bounds = cube.bounds();
    println!("Min corner: ({:.1}, {:.1}, {:.1})",
             bounds.min.x, bounds.min.y, bounds.min.z);
    println!("Max corner: ({:.1}, {:.1}, {:.1})",
             bounds.max.x, bounds.max.y, bounds.max.z);
}
```

**Expected output:**
```
Cube has 8 vertices
Cube has 12 faces
Min corner: (0.0, 0.0, 0.0)
Max corner: (1.0, 1.0, 1.0)
```

**Your task:**
- [ ] Run the code above
- [ ] Modify it to print the center point of the bounding box
- [ ] Calculate the diagonal length of the cube

---

### Exercise 1.3: Explore the Mesh Topology Trait

**Objective:** Understand trait-based polymorphism in the mesh API.

**Steps:**
```rust
use mesh::types::{unit_cube, MeshTopology};

fn print_mesh_info<M: MeshTopology>(mesh: &M) {
    println!("This mesh has {} faces", mesh.face_count());
}

fn main() {
    let cube = unit_cube();
    print_mesh_info(&cube);
}
```

**Your task:**
- [ ] What other methods are available on `MeshTopology`?
- [ ] Create a function that accepts any `MeshTopology` and prints vertex count

---

## Tier 2: Core Operations

**Goal:** Learn mesh repair, validation, and simplification.

### Exercise 2.1: Validate a Mesh

**Objective:** Use the validation system to check mesh quality.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::repair::validate_mesh;

fn main() {
    let cube = unit_cube();
    let report = validate_mesh(&cube);

    println!("Is manifold: {}", report.is_manifold);
    println!("Is watertight: {}", report.is_watertight);
    println!("Boundary edges: {}", report.boundary_edge_count);
    println!("Non-manifold edges: {}", report.non_manifold_edge_count);
}
```

**Expected output:**
```
Is manifold: true
Is watertight: true
Boundary edges: 0
Non-manifold edges: 0
```

**Your task:**
- [ ] Create a mesh with missing faces and see how the report changes
- [ ] What makes a mesh "manifold"?

---

### Exercise 2.2: Repair a Mesh

**Objective:** Use `RepairParams` with builder pattern to configure repairs.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::repair::{repair_mesh, RepairParams};

fn main() {
    let mut mesh = unit_cube();

    // Use builder pattern to configure repair
    let params = RepairParams::default()
        .with_weld_epsilon(1e-6)
        .with_remove_unreferenced(true);

    let summary = repair_mesh(&mut mesh, &params);

    println!("Before: {} vertices", summary.initial_vertices);
    println!("After: {} vertices", summary.final_vertices);
    println!("Welded: {}", summary.vertices_welded);
    println!("Degenerates removed: {}", summary.degenerates_removed);
}
```

**Your task:**
- [ ] Create a mesh with duplicate vertices and repair it
- [ ] Experiment with different `weld_epsilon` values

---

### Exercise 2.3: Simplify a Mesh

**Objective:** Use mesh decimation to reduce polygon count.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::decimate::{decimate_mesh, DecimateParams};

fn main() {
    let cube = unit_cube();
    println!("Original: {} faces", cube.face_count());

    // Reduce to 50% of faces
    let params = DecimateParams::with_target_ratio(0.5);
    let result = decimate_mesh(&cube, &params);

    println!("Simplified: {} faces", result.final_triangles);
    println!("Reduction: {:.1}%",
             (1.0 - result.final_triangles as f64 / cube.face_count() as f64) * 100.0);
}
```

**Your task:**
- [ ] Try `DecimateParams::aggressive()` preset
- [ ] Compare results with `with_preserve_boundary(true)`

---

## Tier 3: Advanced Processing

**Goal:** Master boolean operations, slicing, and mesh offsetting.

### Exercise 3.1: Boolean Union

**Objective:** Combine two meshes using CSG union.

**Steps:**
```rust
use mesh::types::{IndexedMesh, Point3, Vertex, unit_cube};
use mesh::boolean::union;

fn create_offset_cube(offset_x: f64) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    for (x, y, z) in [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0), (0.0, 1.0, 1.0),
    ] {
        mesh.vertices.push(Vertex::new(Point3::new(x + offset_x, y, z)));
    }
    mesh.faces = vec![
        [0,2,1], [0,3,2], [4,5,6], [4,6,7],
        [0,1,5], [0,5,4], [2,3,7], [2,7,6],
        [0,4,7], [0,7,3], [1,2,6], [1,6,5],
    ];
    mesh
}

fn main() {
    let cube_a = unit_cube();
    let cube_b = create_offset_cube(5.0);  // Non-overlapping

    match union(&cube_a, &cube_b) {
        Ok(result) => {
            println!("Union has {} faces", result.mesh.faces.len());
            println!("Stats: faces from A = {}, from B = {}",
                     result.stats.faces_from_a, result.stats.faces_from_b);
        }
        Err(e) => println!("Error: {:?}", e),
    }
}
```

**Expected output:**
```
Union has 24 faces
Stats: faces from A = 12, from B = 12
```

**Your task:**
- [ ] Try overlapping cubes (offset_x = 0.5) and observe the difference
- [ ] Try `difference()` and `intersection()` operations
- [ ] Use `BooleanConfig::for_scans()` preset

---

### Exercise 3.2: Multi-Mesh Operations

**Objective:** Efficiently combine multiple meshes.

**Steps:**
```rust
use mesh::types::{IndexedMesh, Point3, Vertex};
use mesh::boolean::{multi_union, BooleanConfig};

fn main() {
    // Create 3 non-overlapping cubes
    let cubes: Vec<IndexedMesh> = (0..3)
        .map(|i| {
            let offset = i as f64 * 2.0;
            let mut mesh = IndexedMesh::new();
            for (x, y, z) in [
                (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, 1.0), (0.0, 1.0, 1.0),
            ] {
                mesh.vertices.push(Vertex::new(Point3::new(x + offset, y, z)));
            }
            mesh.faces = vec![
                [0,2,1], [0,3,2], [4,5,6], [4,6,7],
                [0,1,5], [0,5,4], [2,3,7], [2,7,6],
                [0,4,7], [0,7,3], [1,2,6], [1,6,5],
            ];
            mesh
        })
        .collect();

    let result = multi_union(&cubes, &BooleanConfig::default()).unwrap();

    println!("Combined {} meshes", result.stats.input_count);
    println!("Result has {} faces", result.mesh.faces.len());
}
```

**Your task:**
- [ ] Time the operation with 10+ meshes
- [ ] Compare `multi_union` vs sequential `union` calls

---

### Exercise 3.3: Mesh Slicing

**Objective:** Generate print layers from a mesh.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::slice::{slice_mesh, SliceParams};

fn main() {
    let cube = unit_cube();
    let result = slice_mesh(&cube, &SliceParams::default());

    println!("Total layers: {}", result.layer_count);
    println!("Estimated print time: {:.1} minutes", result.estimated_print_time);

    // Examine first layer
    if let Some(layer) = result.layers.first() {
        println!("Layer 0 height: {:.3} mm", layer.z_height);
        println!("Layer 0 contours: {}", layer.contours.len());
    }
}
```

**Your task:**
- [ ] Modify `SliceParams` to use different layer heights
- [ ] Count how many layers contain "islands" (disconnected regions)

---

## Tier 4: Analysis & Validation

**Goal:** Use analysis tools for 3D printing preparation.

### Exercise 4.1: Printability Check

**Objective:** Validate a mesh for 3D printing constraints.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::printability::{validate_for_printing, PrinterConfig};

fn main() {
    let cube = unit_cube();

    // Check for FDM printing
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&cube, &config).unwrap();

    println!("Printable: {}", result.is_printable());
    println!("Critical issues: {}", result.critical_count());
    println!("Warnings: {}", result.warning_count());
    println!("\n{}", result.summary());
}
```

**Your task:**
- [ ] Compare FDM vs SLA validation results
- [ ] Create a mesh that exceeds build volume and see the error
- [ ] Use `PrinterConfig::sls_default()` and observe overhang handling

---

### Exercise 4.2: Signed Distance Field

**Objective:** Build an SDF for distance queries.

**Steps:**
```rust
use mesh::types::{unit_cube, Point3};
use mesh::sdf::SignedDistanceField;

fn main() {
    let cube = unit_cube();
    let sdf = SignedDistanceField::new(cube).unwrap();

    // Query points
    let inside = Point3::new(0.5, 0.5, 0.5);
    let outside = Point3::new(2.0, 0.5, 0.5);
    let on_surface = Point3::new(0.0, 0.5, 0.5);

    println!("Distance from center: {:.3}", sdf.distance(inside));
    println!("Distance from outside: {:.3}", sdf.distance(outside));
    println!("Distance from surface: {:.3}", sdf.distance(on_surface));
}
```

**Your task:**
- [ ] What is the sign convention for inside vs outside?
- [ ] Create a grid of points and visualize the distance field

---

### Exercise 4.3: Connected Components

**Objective:** Analyze mesh connectivity.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::repair::components::{find_connected_components, split_into_components};

fn main() {
    let cube = unit_cube();
    let analysis = find_connected_components(&cube);

    println!("Component count: {}", analysis.component_count);
    println!("Largest component: {} faces", analysis.largest_component_size);

    // Split into separate meshes
    let parts = split_into_components(&cube);
    println!("Split into {} meshes", parts.len());
}
```

**Your task:**
- [ ] Create a mesh with 2+ disconnected parts and split them
- [ ] Use `keep_largest_component` to filter small debris

---

## Tier 5: Specialized Operations

**Goal:** Explore advanced features for specific workflows.

### Exercise 5.1: ICP Registration

**Objective:** Align two meshes using iterative closest point.

**Steps:**
```rust
use mesh::types::unit_cube;
use mesh::registration::{icp_align, IcpParams};

fn main() {
    let source = unit_cube();
    let target = unit_cube();  // Same mesh for this test

    let params = IcpParams::default();
    let result = icp_align(&source, &target, &params).unwrap();

    println!("ICP converged in {} iterations", result.iterations);
    println!("Final error: {:.6}", result.mean_squared_error);
}
```

**Your task:**
- [ ] Transform the source mesh and see if ICP aligns it back
- [ ] Experiment with different convergence thresholds

---

### Exercise 5.2: Point Cloud Processing

**Objective:** Work with point cloud data.

**Steps:**
```rust
use mesh::types::Point3;
use mesh::scan::PointCloud;

fn main() {
    // Create a point cloud from positions
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.5, 1.0, 0.0),
        Point3::new(0.5, 0.5, 1.0),
    ];

    let cloud = PointCloud::from_positions(&points);

    println!("Point count: {}", cloud.len());
    println!("Has normals: {}", cloud.has_normals());
}
```

**Your task:**
- [ ] Add colors to the point cloud
- [ ] Convert a mesh to a point cloud and back

---

### Exercise 5.3: Lattice Structures

**Objective:** Generate lattice infill patterns.

**Steps:**
```rust
use mesh::lattice::LatticeParams;

fn main() {
    let params = LatticeParams::default()
        .with_cell_size(5.0);

    println!("Cell size: {:.1} mm", params.cell_size);

    // LatticeParams can be used with lattice generation functions
    // to fill a mesh interior with lightweight structures
}
```

**Your task:**
- [ ] Explore different lattice types available
- [ ] Consider how lattice parameters affect print strength vs material usage

---

## Completion Checklist

Track your progress through the learning path:

### Tier 1: Foundation
- [ ] Exercise 1.1: Create Your First Mesh
- [ ] Exercise 1.2: Use Primitive Generators
- [ ] Exercise 1.3: Explore the Mesh Topology Trait

### Tier 2: Core Operations
- [ ] Exercise 2.1: Validate a Mesh
- [ ] Exercise 2.2: Repair a Mesh
- [ ] Exercise 2.3: Simplify a Mesh

### Tier 3: Advanced Processing
- [ ] Exercise 3.1: Boolean Union
- [ ] Exercise 3.2: Multi-Mesh Operations
- [ ] Exercise 3.3: Mesh Slicing

### Tier 4: Analysis & Validation
- [ ] Exercise 4.1: Printability Check
- [ ] Exercise 4.2: Signed Distance Field
- [ ] Exercise 4.3: Connected Components

### Tier 5: Specialized Operations
- [ ] Exercise 5.1: ICP Registration
- [ ] Exercise 5.2: Point Cloud Processing
- [ ] Exercise 5.3: Lattice Structures

---

## Next Steps

After completing this learning path:

1. **Explore the API docs**: Run `cargo doc --open` in the mesh directory
2. **Read the examples**: Check `mesh/examples/` for more complex workflows
3. **Run the test suite**: `cargo test` shows expected behavior for edge cases
4. **Check api_regression.rs**: See `mesh/mesh/tests/api_regression.rs` for API patterns

## Getting Help

- File issues on GitHub for bugs
- Check the crate-level documentation for each module
- The `prelude` module provides common imports for quick starts
