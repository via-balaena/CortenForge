# Requirements Traceability

> **Purpose**: Link requirements → tests → code for safety-critical traceability.
> Required for ISO 26262, IEC 62304, DO-178C compliance.

## Overview

This directory contains structured requirements that trace to:
- **Tests**: Each requirement links to test functions that verify it
- **Code**: Each requirement links to implementation modules
- **Risks**: Each requirement traces to risk mitigations

## Directory Structure

```
requirements/
├── README.md           # This file
├── schema.yaml         # Requirement schema definition
├── mesh/               # Mesh domain requirements
│   ├── REQ-MESH-001.yaml
│   └── ...
├── sensor/             # Sensor domain requirements
├── ml/                 # ML domain requirements
└── safety/             # Cross-cutting safety requirements
```

## Requirement Format

Each requirement is a YAML file following `schema.yaml`:

```yaml
id: REQ-MESH-001
title: Mesh Topology Validation
category: functional
priority: HIGH
status: implemented

description: |
  The system shall detect non-manifold meshes including:
  - Open (boundary) edges
  - Self-intersecting triangles
  - Degenerate triangles (zero area)
  - Duplicate triangles

rationale: |
  Non-manifold meshes cause failures in downstream operations
  (boolean, shell generation, 3D printing).

verification:
  method: test
  tests:
    - crate: mesh-repair
      function: tests::test_detect_open_edges
    - crate: mesh-repair
      function: tests::test_detect_self_intersection
    - crate: mesh-repair
      function: tests::test_detect_degenerate

implementation:
  - crate: mesh-repair
    module: validate

traces_to:
  - RISK-001  # Manufacturing failure from invalid geometry

traces_from:
  - SYS-001   # System-level geometry processing requirement

history:
  - date: 2026-01-18
    author: CortenForge Team
    change: Initial requirement
```

## Verification Methods

| Method | Description | Evidence |
|--------|-------------|----------|
| `test` | Unit/integration test | Test passes |
| `review` | Code review | Review record |
| `analysis` | Static analysis | Analysis report |
| `inspection` | Manual inspection | Inspection record |

## Generating Traceability Matrix

```bash
# Generate traceability matrix (future xtask command)
cargo xtask trace

# Output: traceability-matrix.md
# Shows: REQ → TEST → CODE → RISK
```

## Verification Status

| Status | Meaning |
|--------|---------|
| `draft` | Requirement not yet finalized |
| `approved` | Requirement approved, not implemented |
| `implemented` | Code exists, tests not complete |
| `verified` | All verification tests pass |
| `deprecated` | No longer applicable |

## CI Integration

The CI pipeline will (future):
1. Parse all requirement YAML files
2. Verify each linked test exists and passes
3. Generate traceability matrix artifact
4. Flag requirements without tests
5. Flag tests without requirements

## For Safety-Critical Applications

When targeting certification (ISO 26262, IEC 62304, DO-178C):

1. **Complete Traceability**: Every requirement must trace to tests
2. **Bidirectional Links**: Tests must trace back to requirements
3. **Coverage Analysis**: All requirements must be tested
4. **Change Impact**: Changes flag affected requirements
5. **Audit Trail**: Version history for each requirement

## Getting Started

1. Create requirement file in appropriate subdirectory
2. Fill in all required fields per schema
3. Add test annotations in code:
   ```rust
   /// Verifies: REQ-MESH-001
   #[test]
   fn test_detect_open_edges() { ... }
   ```
4. Run `cargo xtask trace` to validate links
