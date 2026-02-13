# Custom-Fit Shoe — Phase 1 Product Spec

Scan-to-print pipeline for a fully custom 3D-printed shoe built from a raw foot scan.

## Pipeline

```
Foot scan (PLY/STL) ─► Mesh cleanup ─► Sole geometry (parametric curves)
    ─► Lattice midsole (tunable compliance) ─► Printability validation ─► Export (STL/3MF)
```

1. **Scan ingestion** — Load a 3D foot scan point cloud or mesh from PLY or STL.
2. **Mesh cleanup** — Repair non-manifold edges, fill holes, remove noise, and produce a watertight surface.
3. **Sole geometry from parametric curves** — Define the outsole and upper profiles using B-spline / NURBS curves, then generate the sole surface mesh.
4. **Lattice midsole** — Fill the midsole region with a density-graded lattice structure for tunable stiffness and energy return.
5. **Printability validation** — Check wall thickness, overhang angles, support requirements, and watertightness.
6. **Export** — Write the final mesh as STL or 3MF for slicing and printing.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `mesh-scan` | Ingest raw 3D foot scan data |
| `mesh-repair` | Repair non-manifold geometry, fill holes, denoise |
| `mesh-boolean` | Boolean union of sole and upper components |
| `mesh-offset` | Wall-thickness offset for shell generation |
| `mesh-shell` | Generate thin-wall shell geometry for the shoe upper |
| `mesh-lattice` | Create density-graded lattice fills for the midsole |
| `mesh-from-curves` | Generate sole surfaces from parametric curve profiles |
| `curve-types` | B-spline / NURBS curve definitions for sole profiles |
| `mesh-printability` | Validate printability constraints (thickness, overhangs) |
| `mesh-io` | Read/write PLY, STL, 3MF mesh formats |
| `mesh-types` | Core mesh data structures |

## Input

- 3D foot scan as a PLY or STL point cloud / triangle mesh.

## Output

- Printable shoe mesh exported as STL or 3MF, ready for slicing.

## Manufacturing

- **Primary:** FDM 3D printing (TPU filament for flexible midsole, rigid PLA/PETG for outsole).
- **Alternative:** SLS 3D printing (Nylon PA12 for single-material production).

## Acceptance Criteria

1. Output mesh is watertight (zero non-manifold edges, zero open boundaries).
2. Lattice midsole has a measurable density gradient (heel-to-toe or pressure-based).
3. All regions pass printability checks (minimum wall thickness >= 0.8 mm, overhang angles <= 45 deg without support).
4. Pipeline completes without panics on a reference scan dataset.
5. Exported 3MF/STL is loadable by at least one mainstream slicer (PrusaSlicer, Cura).

## Status

**Spec** — not yet implemented.
