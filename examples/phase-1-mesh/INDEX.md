# Phase 1: Mesh Pipeline

> **Scan -> Design -> Manufacture**

The first phase of CortenForge product development. Three products that
exercise the full scan-to-manufacture mesh pipeline across different
geometries and manufacturing methods. No simulation, no sensors -- just
geometry done right.

---

## New Domains Introduced

| Domain | Purpose |
|--------|---------|
| mesh-scan | Point cloud and scan data ingestion |
| mesh-repair | Watertight mesh healing, hole filling, normal repair |
| mesh-boolean | CSG operations (union, difference, intersection) |
| mesh-offset | Uniform and variable wall offsetting |
| mesh-shell | Thin-walled shell extraction |
| mesh-lattice | Volumetric lattice infill generation |
| mesh-printability | Manufacturing constraint validation |
| mesh-io | Import/export (STL, OBJ, 3MF, STEP) |
| curve-types | Parametric curve primitives (Bezier, B-spline, NURBS) |
| mesh-from-curves | Surface generation from curve networks |
| mesh-morph | Freeform mesh deformation and morphing |
| mesh-slice | Layer slicing for additive manufacturing |

**Prerequisites:** None. This is the first phase.

---

## Products

| Product | Description | Manufacturing | Spec |
|---------|-------------|---------------|------|
| Custom Shoe | 3D foot scan -> parametric sole -> lattice midsole -> printable shoe | FDM/SLS 3D print | [SPEC.md](./custom-shoe/SPEC.md) |
| Orthotic Insole | Foot scan + pressure map -> variable-density lattice insole | FDM/SLS 3D print or CNC | [SPEC.md](./orthotic-insole/SPEC.md) |
| Ergonomic Handle | Hand scan -> grip cavity -> offset walls -> ergonomic tool handle | FDM 3D print | [SPEC.md](./ergonomic-handle/SPEC.md) |

**Goal:** Prove the full scan-to-manufacture pipeline. Every mesh crate
must be exercised by at least two of the three products. If a crate fails
under any of these geometries, we fix the crate -- not the geometry.

---

## Crate Coverage

| Crate | Shoe | Insole | Handle |
|-------|------|--------|--------|
| mesh-scan | x | x | x |
| mesh-repair | x | x | x |
| mesh-boolean | x | | x |
| mesh-offset | x | | x |
| mesh-shell | x | x | |
| mesh-lattice | x | x | |
| mesh-printability | x | x | x |
| mesh-io | x | x | x |
| mesh-slice | | x | |
| mesh-morph | | | x |
| mesh-from-curves | x | | x |
| curve-types | x | | x |

---

## Running

```bash
# Custom shoe: scan -> sole -> lattice -> STL
cargo run -p example-custom-shoe

# Orthotic insole: scan + pressure -> variable lattice -> STL
cargo run -p example-orthotic-insole

# Ergonomic handle: hand scan -> cavity -> offset -> STL
cargo run -p example-ergonomic-handle
```

---

## Shared Learnings

Things we expect to discover and codify during Phase 1:

- Scan cleanup thresholds that work across foot and hand geometry
- Boolean robustness under degenerate triangles and near-coplanar faces
- Lattice density gradients that are both structurally sound and printable
- Offset wall behavior at sharp concavities and thin features
- Minimum viable printability checks (wall thickness, overhang angle, bridging distance)
- Import/export round-trip fidelity (STL -> internal -> STL should be lossless)

---

## Completion Criteria

Phase 1 is complete when all of the following hold:

- [ ] All three products produce watertight, manifold output meshes
- [ ] Every mesh passes `mesh-printability` validation with zero warnings
- [ ] STL export round-trips without vertex drift or face inversion
- [ ] Lattice infill density is continuously variable (not just uniform)
- [ ] Boolean operations handle all three product geometries without crashes or degenerate output
- [ ] Each product pipeline runs end-to-end from scan data to print-ready file in a single `cargo run` invocation
- [ ] At least one product has been physically manufactured from its output

---

*See also: [Product Roadmap](../PRODUCT_ROADMAP.md) | [Phase 2: Mechanism Design](../phase-2-mechanism/INDEX.md)*
