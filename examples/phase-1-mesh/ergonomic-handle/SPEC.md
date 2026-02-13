# Ergonomic Handle — Phase 1 Product Spec

Custom ergonomic tool handle from a 3D hand scan with boolean-carved finger channels and curve-blended surfaces.

## Pipeline

```
Hand scan (PLY/STL) ─► Mesh cleanup ─► Base handle from curves
    ─► Boolean subtraction (finger channels) ─► Offset (wall thickness)
    ─► Surface morphing (ergonomic blend) ─► Printability validation ─► Export (STL/3MF)
```

1. **Scan ingestion** — Load a 3D hand scan from PLY or STL.
2. **Mesh cleanup** — Repair non-manifold edges, fill holes, denoise the scan.
3. **Base handle geometry** — Generate the cylindrical handle body from parametric curves (profile curves, cross-section sweeps).
4. **Boolean subtraction** — Carve finger channels into the handle using boolean difference with shaped volumes derived from the hand scan.
5. **Offset for wall thickness** — Apply inward offset to create a hollow handle with controlled wall thickness.
6. **Surface morphing** — Blend the carved handle surface with ergonomic target shapes for smooth grip contours.
7. **Printability validation** — Verify wall thickness, overhang angles, and structural integrity.
8. **Export** — Write the final handle mesh as STL or 3MF.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `mesh-scan` | Ingest raw 3D hand scan data |
| `mesh-repair` | Repair non-manifold geometry, fill holes, denoise |
| `mesh-boolean` | Boolean subtraction to carve finger channels |
| `mesh-offset` | Inward offset for hollow wall-thickness control |
| `mesh-morph` | Blend carved surfaces with ergonomic target shapes |
| `mesh-from-curves` | Generate base handle body from profile curves |
| `curve-types` | Curve definitions for handle profiles and cross-sections |
| `mesh-printability` | Validate printability constraints |
| `mesh-io` | Read/write PLY, STL, 3MF mesh formats |
| `mesh-types` | Core mesh data structures |

## Input

- 3D hand scan as a PLY or STL point cloud / triangle mesh.

## Output

- Printable ergonomic handle mesh exported as STL or 3MF.

## Manufacturing

- **Primary:** FDM 3D printing (PLA or PETG for rigid handles, TPU overmold for soft-touch grip zones).

## Key Technical Validations

- **`mesh-boolean` subtraction:** Confirms that boolean difference produces clean, manifold results when carving organic shapes from a cylindrical body.
- **`mesh-offset` wall thickness:** Validates that inward offset preserves topology and maintains minimum thickness in thin regions around finger channels.
- **`mesh-morph` surface blending:** Verifies smooth interpolation between the carved handle and ergonomic target shapes without self-intersections.

## Acceptance Criteria

1. Output mesh is watertight with zero non-manifold edges.
2. Finger channels are cleanly carved (no residual geometry, no self-intersections at boolean boundaries).
3. Wall thickness >= 1.2 mm everywhere after offset.
4. Morphed surfaces are C0-continuous or better (no visible faceting at blend boundaries).
5. All regions pass printability checks (overhang angles <= 45 deg without support).
6. Pipeline completes without panics on a reference hand scan dataset.
7. Exported mesh is loadable by at least one mainstream slicer.

## Status

**Spec** — not yet implemented.
