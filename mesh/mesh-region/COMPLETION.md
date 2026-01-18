# mesh-region Completion

## Status: Complete

## Summary
Region selection, material zones, and thickness mapping for triangle meshes. Provides tools for defining and working with regions on a mesh, enabling variable thickness, material zones, and selective operations. Use cases include defining thick heel cups on skate boots, marking ventilation zones on helmets, and specifying material transitions in multi-material prints.

## Public API
- `MeshRegion` - Named subset of mesh vertices/faces
- `RegionMap` - Collection of named regions
- `RegionSelector` - Criteria for selecting vertices/faces (sphere, half-space, box, etc.)
- `FloodFillCriteria` - Criteria for flood-fill selection
- `ThicknessMap` - Per-vertex/face thickness values
- `MaterialZone` - Material assignment for a region
- `MaterialMap` - Collection of material zones
- `MaterialProperties` - Material properties (shore hardness, color, etc.)
- `RegionError` / `RegionResult` - Error handling types

## Test Coverage
- Unit tests: 47 tests in src/
- Integration tests: None (tests in tests/ directory)
- Doc tests: All public items have examples

## Quality
- Zero clippy warnings
- Zero doc warnings
- No unwrap/expect in library code (enforced via `#![deny(clippy::unwrap_used)]`)

## Dependencies
- mesh-types (workspace)
- nalgebra (workspace)
- thiserror (workspace)
- hashbrown (workspace)
