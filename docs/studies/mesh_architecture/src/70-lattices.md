# mesh-lattice — TPMS, density maps, infill

What this part covers (when authored to depth):

## Scope and tier

`mesh-lattice` is L0 — generates lattice geometry within bounding volumes for 3D-printing-style infill. Output is `IndexedMesh` (triangulated lattice surface) ready for the rest of the ecosystem.

Six lattice families are supported:

| Family | Type | Strength | Self-supporting | Use case |
|---|---|---|---|---|
| **Cubic** | strut grid | low | yes | simplest, easy to print |
| **Octet-Truss** | strut graph | high | yes | high strength-to-weight ratio |
| **Gyroid** | TPMS | medium-high | yes | smooth, organic, bio-inspired |
| **Schwarz-P** | TPMS | medium | yes | symmetric variant of gyroid |
| **Diamond** | TPMS | medium-high | yes | mechanically isotropic |
| **Voronoi** | irregular cell | varies | varies | organic / random-looking |

TPMS (Triply Periodic Minimal Surface) lattices are the contemporary workhorses — smooth and self-supporting, naturally extracted via marching cubes from analytic implicit functions. Strut-based lattices (cubic, octet-truss) are computationally cheaper but less mechanically isotropic.

## Density maps for variable-density lattices

```rust
let density = DensityMap::Gradient {
    from:          Point3::new(0.0, 0.0, 0.0),
    from_density:  0.1,
    to:            Point3::new(0.0, 0.0, 100.0),
    to_density:    0.5,
};
let params = LatticeParams::gyroid(8.0).with_density_map(density);
```

Density maps modulate lattice cell size or strut diameter spatially. Common patterns:

- **Gradient** — density varies linearly between two points. Useful for stiffness-graded parts (denser at load-bearing regions).
- **Constant** — uniform density everywhere; the default.
- **Custom** (planned) — function pointer or closure for arbitrary density distribution.

The depth pass covers: the density-to-strut-thickness or density-to-TPMS-isovalue mapping per lattice family; the resolution implications (density gradients require finer marching-cubes grids to capture without aliasing); the manufacturability tradeoff (denser lattices are stronger but slower to print and harder to remove powder/resin from).

## Lattice + shell composition

The most useful lattice pattern combines `mesh-lattice` with `mesh-shell`:

1. Generate a lattice within a bounding box (`generate_lattice`).
2. Generate a shell from the part geometry (`ShellBuilder`).
3. Boolean-combine: keep lattice triangles inside the shell's hollow region, keep shell surface as the outer wall.
4. Result: a printable part with TPMS infill and solid outer skin.

This pattern is the canonical "lightweight rigid shell" workflow in additive manufacturing. The depth pass walks through it as a worked example, with code, and discusses the open question of whether the boolean composition deserves its own first-class crate operation (currently it's a multi-step recipe; a dedicated `compose_shell_and_lattice` would streamline it).

## Relevance to soft-body and casting

**For soft-body**: lattices currently aren't load-bearing. The layered silicone device's middle shell uses a continuous carbon-black-loaded composite, modeled as a uniform material with elevated stiffness. *Future possibility*: explicitly modeling the carbon-mesh reinforcement as a lattice geometry instead of a continuous medium. Would require lattice-aware meshing and material assignment in `sim-soft`. Not on the current roadmap; flagged as an option in [Part 10 roadmap](100-roadmap.md).

**For casting**: lattice infill within a mold is exotic but possible — sacrificial lattice cores (PVA-printed gyroid, dissolved out post-cast) for hollow parts with internal structure. More realistically, lattice-as-design-output: the casting domain delivers fab-ready files; if the part design includes lattice features, those features need to round-trip through the casting toolchain without being treated as geometric noise. The depth pass discusses this case.

## What's NOT in `mesh-lattice`

- **Mechanical properties prediction.** Computing the effective Young's modulus of a gyroid lattice as a function of density and cell size — that's a homogenization problem; out of scope. Some literature provides closed-form approximations for specific families; integrating them would be a future convenience feature, not core.
- **Parametric lattice authoring.** The crate generates lattices from parameters; it doesn't expose a *parametric editor* over lattice geometry. That's a `cf-design` concern.
- **Conformal lattices.** Lattice cells that adapt to a non-orthogonal bounding shape (e.g., a gyroid that follows the curvature of an ergonomic handle). Hard problem; out of current scope.
- **Lattice optimization.** Finding the lattice configuration that minimizes some objective (mass under stiffness constraint, etc.). Topology-optimization territory; out of scope.

The depth pass enumerates these and discusses where each would live if implemented.
