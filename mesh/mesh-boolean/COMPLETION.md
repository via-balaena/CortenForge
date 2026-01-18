# mesh-boolean Completion

## Status: Complete

## Summary
Boolean operations (CSG) for triangle meshes. Provides constructive solid geometry operations including union, intersection, and difference. Features BVH-accelerated intersection detection, edge insertion for clean boundaries, coplanar face handling, multi-mesh operations with parallel tree reduction, and optional GPU acceleration.

## Public API
- `union()` / `union_with_config()` - Combine meshes (A + B)
- `intersection()` / `intersection_with_config()` - Overlap meshes (A & B)
- `difference()` / `difference_with_config()` - Subtract meshes (A - B)
- `boolean_operation()` - Generic boolean operation
- `multi_union()` - Parallel tree-based union of multiple meshes
- `multi_intersection()` - Intersection of multiple meshes
- `sequential_difference()` - Sequential difference operations
- `concatenate_meshes()` - Simple mesh concatenation
- `BooleanConfig` - Configuration with presets (default, for_scans, for_cad, strict)
- `BooleanOp` - Operation type enumeration
- `CleanupLevel` - Post-operation cleanup level
- `CoplanarStrategy` - Strategy for handling coplanar faces
- `BooleanOperationResult` / `BooleanStats` - Operation results and statistics
- `MultiMeshResult` / `MultiMeshStats` - Multi-mesh operation results
- `BooleanError` / `BooleanResult` - Error handling types
- `prelude` module - Convenient imports

## Test Coverage
- Unit tests: 95 tests in src/
- Integration tests: None (tests in tests/ directory)
- Doc tests: All public items have examples

## Quality
- Zero clippy warnings
- Zero doc warnings
- No unwrap/expect in library code (enforced via `#![deny(clippy::unwrap_used)]`)

## Dependencies
- mesh-types (workspace)
- mesh-repair (workspace)
- nalgebra (workspace)
- thiserror (workspace)
- rayon (workspace)
- hashbrown (workspace)
- smallvec (workspace)
- wgpu (optional, for GPU feature)
- bytemuck (optional, for GPU feature)
- pollster (optional, for GPU feature)
