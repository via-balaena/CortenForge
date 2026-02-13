# Orthotic Insole — Phase 1 Product Spec

Custom orthotic insole from a 3D foot scan with pressure-map-driven variable-density lattice.

## Pipeline

```
Foot scan (PLY/STL) + Pressure map ─► Mesh cleanup ─► Shell generation
    ─► Variable-density lattice fill ─► Layer slicing analysis ─► Printability validation ─► Export (STL/3MF)
```

1. **Scan ingestion** — Load a 3D foot scan from PLY or STL.
2. **Pressure map overlay** — Map plantar pressure data onto the foot surface to identify high-load zones (heel, metatarsal heads, arch).
3. **Mesh cleanup** — Repair geometry, fill holes, ensure watertight surface.
4. **Shell generation** — Create the insole shell with correct wall thickness for structural integrity.
5. **Variable-density lattice fill** — Generate a lattice structure whose density varies by zone: denser under high-pressure areas for support, sparser in low-pressure areas for cushioning.
6. **Layer slicing analysis** — Slice the insole into layers and analyze each for printability and structural continuity.
7. **Printability validation** — Verify wall thickness, unsupported spans, and layer adhesion requirements.
8. **Export** — Write the final insole mesh as STL or 3MF.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `mesh-scan` | Ingest raw 3D foot scan data |
| `mesh-repair` | Repair non-manifold geometry, fill holes |
| `mesh-shell` | Generate insole shell with controlled wall thickness |
| `mesh-lattice` | Variable-density lattice based on pressure zones |
| `mesh-slice` | Layer-by-layer slicing for print analysis |
| `mesh-printability` | Validate printability constraints |
| `mesh-io` | Read/write PLY, STL, 3MF mesh formats |
| `mesh-types` | Core mesh data structures |

## Input

- 3D foot scan as a PLY or STL point cloud / triangle mesh.
- Plantar pressure map data (per-vertex scalar field or CSV grid).

## Output

- Printable orthotic insole mesh exported as STL or 3MF.

## Manufacturing

- **Primary:** FDM 3D printing (TPU for flexible cushioning, PETG for rigid support).
- **Alternative:** SLS 3D printing (Nylon PA12 for uniform material properties).
- **Alternative:** CNC milling from EVA foam blanks (subtractive, for simpler geometries).

## Key Technical Validations

- **`mesh-lattice` density gradient API:** Confirms that the lattice generator accepts a spatially-varying density field and produces correct cell sizing across zones.
- **`mesh-slice` layer analysis:** Validates that sliced layers have no disconnected islands and meet minimum feature size per layer.

## Acceptance Criteria

1. Output mesh is watertight with zero non-manifold edges.
2. Lattice density correlates with input pressure map (denser in high-pressure zones, measurable via cell count per zone).
3. Sliced layers show no disconnected islands or sub-minimum features.
4. All regions pass printability checks (wall thickness >= 0.6 mm, unsupported spans <= 8 mm).
5. Pipeline completes without panics on a reference scan + pressure dataset.
6. Exported mesh is loadable by at least one mainstream slicer.

## Status

**Spec** — not yet implemented.
