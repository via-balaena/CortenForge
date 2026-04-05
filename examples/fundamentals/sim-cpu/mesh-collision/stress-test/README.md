# Stress Test — Headless Validation

Headless (no Bevy) validation of all 7 mesh collision pairs plus edge cases.
Tests every dispatch path in `mesh_collide.rs`: plane, sphere, box, capsule,
cylinder, ellipsoid, and mesh-mesh. Exits with code 1 if any check fails.

## Checks

| # | Group | Check | Tolerance |
|---|-------|-------|-----------|
| 1 | Mesh-Plane | Tetrahedron rest height = centroid | < 3mm |
| 2 | Mesh-Plane | Contact force / weight = 1.0 | 5% |
| 3 | Mesh-Plane | Contacts exist (ncon >= 1) | -- |
| 4 | Mesh-Sphere | Sphere rests at z = radius | < 3mm |
| 5 | Mesh-Sphere | Contact force / weight = 1.0 | 5% |
| 6 | Mesh-Box | Box rests at z = half_z | < 3mm |
| 7 | Mesh-Box | Contact force / weight = 1.0 | 5% |
| 8 | Mesh-Capsule | Sideways capsule rests at z = radius | < 3mm |
| 9 | Mesh-Capsule | Contact force / weight = 1.0 | 5% |
| 10 | Mesh-Cylinder | Cylinder rests at z = half_length (GJK/EPA) | < 3mm |
| 11 | Mesh-Cylinder | Contact force / weight = 1.0 | 5% |
| 12 | Mesh-Cylinder | MULTICCD produces >= 2 contacts | -- |
| 13 | Mesh-Ellipsoid | Oblate ellipsoid rests at z = rz = 0.05 | < 3mm |
| 14 | Mesh-Ellipsoid | z < 0.1 (rejects sphere approximation) | -- |
| 15 | Mesh-Ellipsoid | Contact force / weight = 1.0 | 5% |
| 16 | Mesh-Mesh | Wedge on platform settled (vz < 0.01) | -- |
| 17 | Mesh-Mesh | Wedge above platform surface | -- |
| 18 | Mesh-Mesh | Contact force / weight = 1.0 | 10% |
| 19 | Edge: Separated | Two meshes far apart: ncon = 0 | exact |
| 20 | Edge: Single tri | 1-triangle mesh vs sphere: no crash, contact produced | -- |
| 21 | Edge: Swapped order | Primitive-mesh (sphere first): same result | < 1mm |

## Run

```
cargo run -p example-mesh-collision-stress-test --release
```

Exit code 0 = all pass, exit code 1 = failure.
