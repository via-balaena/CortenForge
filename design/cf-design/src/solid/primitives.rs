//! Primitive, path-based, and bio-inspired constructors for [`Solid`].

use nalgebra::{Point3, Vector3};

use super::Solid;
use crate::field_node::{FieldNode, Val};

impl Solid {
    // ── Primitive constructors ────────────────────────────────────────

    /// Sphere centered at origin with the given radius.
    ///
    /// Exact SDF.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "sphere radius must be positive and finite, got {radius}"
        );
        Self {
            node: FieldNode::Sphere {
                radius: Val::from(radius),
            },
        }
    }

    /// Axis-aligned box centered at origin with the given half-extents.
    ///
    /// Exact SDF.
    ///
    /// # Panics
    ///
    /// Panics if any half-extent is not positive and finite.
    #[must_use]
    pub fn cuboid(half_extents: Vector3<f64>) -> Self {
        assert!(
            half_extents.iter().all(|&v| v > 0.0 && v.is_finite()),
            "cuboid half_extents must be positive and finite, got {half_extents:?}"
        );
        Self {
            node: FieldNode::Cuboid { half_extents },
        }
    }

    /// Z-aligned cylinder centered at origin.
    ///
    /// Exact SDF. The cylinder extends from `z = -half_height` to
    /// `z = half_height` with the given radius.
    ///
    /// # Panics
    ///
    /// Panics if `radius` or `half_height` is not positive and finite.
    #[must_use]
    pub fn cylinder(radius: f64, half_height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "cylinder radius must be positive and finite, got {radius}"
        );
        assert!(
            half_height > 0.0 && half_height.is_finite(),
            "cylinder half_height must be positive and finite, got {half_height}"
        );
        Self {
            node: FieldNode::Cylinder {
                radius,
                half_height,
            },
        }
    }

    /// Z-aligned capsule (cylinder with hemispherical caps) centered at origin.
    ///
    /// Exact SDF. The cylindrical segment extends from `z = -half_height` to
    /// `z = half_height`. Total height is `2 * (half_height + radius)`.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite, or `half_height` is
    /// negative or non-finite.
    #[must_use]
    pub fn capsule(radius: f64, half_height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "capsule radius must be positive and finite, got {radius}"
        );
        assert!(
            half_height >= 0.0 && half_height.is_finite(),
            "capsule half_height must be non-negative and finite, got {half_height}"
        );
        Self {
            node: FieldNode::Capsule {
                radius,
                half_height,
            },
        }
    }

    /// Ellipsoid centered at origin with the given axis radii.
    ///
    /// **Not an exact SDF** — the field magnitude is approximate, but the
    /// zero-isosurface is correct. Safe for meshing; `shell()` and `round()`
    /// will produce non-uniform results.
    ///
    /// # Panics
    ///
    /// Panics if any radius is not positive and finite.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        assert!(
            radii.iter().all(|&v| v > 0.0 && v.is_finite()),
            "ellipsoid radii must be positive and finite, got {radii:?}"
        );
        Self {
            node: FieldNode::Ellipsoid { radii },
        }
    }

    /// Torus in the XY plane centered at origin.
    ///
    /// Exact SDF. `major` is the distance from the center to the tube center.
    /// `minor` is the tube radius.
    ///
    /// # Panics
    ///
    /// Panics if `major` or `minor` is not positive and finite.
    #[must_use]
    pub fn torus(major: f64, minor: f64) -> Self {
        assert!(
            major > 0.0 && major.is_finite(),
            "torus major radius must be positive and finite, got {major}"
        );
        assert!(
            minor > 0.0 && minor.is_finite(),
            "torus minor radius must be positive and finite, got {minor}"
        );
        Self {
            node: FieldNode::Torus { major, minor },
        }
    }

    /// Cone with apex at origin, expanding downward along -Z.
    ///
    /// Exact SDF. The base is at `z = -height` with the given `radius`.
    ///
    /// # Panics
    ///
    /// Panics if `radius` or `height` is not positive and finite.
    #[must_use]
    pub fn cone(radius: f64, height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "cone radius must be positive and finite, got {radius}"
        );
        assert!(
            height > 0.0 && height.is_finite(),
            "cone height must be positive and finite, got {height}"
        );
        Self {
            node: FieldNode::Cone { radius, height },
        }
    }

    /// Half-space defined by a plane. Points on the `normal` side are outside
    /// (positive); points on the opposite side are inside (negative).
    ///
    /// Exact SDF when `normal` is unit length.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero-length or non-finite, or `offset` is
    /// non-finite.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, offset: f64) -> Self {
        let len = normal.norm();
        assert!(
            len > 1e-12 && normal.iter().all(|v| v.is_finite()),
            "plane normal must be non-zero and finite, got {normal:?}"
        );
        assert!(
            offset.is_finite(),
            "plane offset must be finite, got {offset}"
        );
        // Normalize the normal to ensure exact SDF property.
        let unit_normal = normal / len;
        let scaled_offset = offset / len;
        Self {
            node: FieldNode::Plane {
                normal: unit_normal,
                offset: scaled_offset,
            },
        }
    }

    // ── Path-based primitives ────────────────────────────────────────

    /// Pipe along a polyline path with spherical cross-section.
    ///
    /// Exact SDF. The pipe follows the straight-line segments connecting the
    /// vertices with the given radius. Corners are naturally rounded via the
    /// min-of-segments formulation.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 vertices, if `radius` is not positive and finite,
    /// or if any vertex coordinate is non-finite.
    #[must_use]
    pub fn pipe(vertices: Vec<Point3<f64>>, radius: f64) -> Self {
        assert!(
            vertices.len() >= 2,
            "pipe requires at least 2 vertices, got {}",
            vertices.len()
        );
        assert!(
            radius > 0.0 && radius.is_finite(),
            "pipe radius must be positive and finite, got {radius}"
        );
        assert!(
            vertices
                .iter()
                .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
            "pipe vertices must have finite coordinates"
        );
        Self {
            node: FieldNode::Pipe { vertices, radius },
        }
    }

    /// Pipe along a Catmull-Rom spline with spherical cross-section.
    ///
    /// Near-exact SDF. The spline smoothly interpolates through the control
    /// points. Uses Catmull-Rom interpolation with open-curve endpoint
    /// handling.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 control points, if `radius` is not positive and
    /// finite, or if any control point coordinate is non-finite.
    #[must_use]
    pub fn pipe_spline(control_points: Vec<Point3<f64>>, radius: f64) -> Self {
        assert!(
            control_points.len() >= 2,
            "pipe_spline requires at least 2 control points, got {}",
            control_points.len()
        );
        assert!(
            radius > 0.0 && radius.is_finite(),
            "pipe_spline radius must be positive and finite, got {radius}"
        );
        assert!(
            control_points
                .iter()
                .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
            "pipe_spline control points must have finite coordinates"
        );
        Self {
            node: FieldNode::PipeSpline {
                control_points,
                radius,
            },
        }
    }

    /// Loft along Z axis with variable circular cross-section.
    ///
    /// Each `(z, radius)` pair defines a cross-section station. The radius
    /// is cubic-interpolated (Catmull-Rom) between stations, producing smooth
    /// transitions. The shape is capped at both ends.
    ///
    /// **Approximate SDF**: exact for constant-radius sections (equivalent to
    /// a cylinder), approximate in tapered regions where |dR/dz| > 0.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 stations, if stations are not strictly sorted
    /// by Z, or if any radius is not positive and finite.
    #[must_use]
    pub fn loft(stations: &[(f64, f64)]) -> Self {
        assert!(
            stations.len() >= 2,
            "loft requires at least 2 stations, got {}",
            stations.len()
        );
        for (i, &(z, r)) in stations.iter().enumerate() {
            assert!(z.is_finite(), "loft station {i} z must be finite, got {z}");
            assert!(
                r > 0.0 && r.is_finite(),
                "loft station {i} radius must be positive and finite, got {r}"
            );
            if i > 0 {
                assert!(
                    z > stations[i - 1].0,
                    "loft stations must be strictly sorted by z: station {i} z={z} \
                     <= station {} z={}",
                    i - 1,
                    stations[i - 1].0
                );
            }
        }
        let internal: Vec<[f64; 2]> = stations
            .iter()
            .map(|&(z, r)| <[f64; 2]>::from((z, r)))
            .collect();
        Self {
            node: FieldNode::Loft { stations: internal },
        }
    }

    // ── Bio-inspired primitives ────────────────────────────────────────

    /// Superellipsoid centered at origin — tunable between box, cylinder,
    /// sphere, and diamond.
    ///
    /// **Approximate SDF** (like `ellipsoid`). The zero-isosurface is correct
    /// but distance magnitude is approximate. Safe for meshing; `shell()` and
    /// `round()` will produce non-uniform results.
    ///
    /// - `n1 = n2 = 2` → ellipsoid
    /// - `n1 = n2 → ∞` → cuboid
    /// - `n1 = n2 = 1` → octahedron
    /// - `n1 = 2, n2 → ∞` → cylinder
    ///
    /// # Panics
    ///
    /// Panics if any radius is not positive and finite, or if `n1`/`n2` are
    /// not positive and finite.
    #[must_use]
    pub fn superellipsoid(radii: Vector3<f64>, n1: f64, n2: f64) -> Self {
        assert!(
            radii.iter().all(|&v| v > 0.0 && v.is_finite()),
            "superellipsoid radii must be positive and finite, got {radii:?}"
        );
        assert!(
            n1 > 0.0 && n1.is_finite(),
            "superellipsoid n1 must be positive and finite, got {n1}"
        );
        assert!(
            n2 > 0.0 && n2.is_finite(),
            "superellipsoid n2 must be positive and finite, got {n2}"
        );
        Self {
            node: FieldNode::Superellipsoid { radii, n1, n2 },
        }
    }

    /// Logarithmic spiral tube in the XY plane.
    ///
    /// The spiral curve `r(θ) = a · exp(b · θ)` is traced from `θ = 0` to
    /// `θ = turns · 2π`, producing a tube of the given thickness.
    ///
    /// - `a` — initial radius (at θ=0)
    /// - `b` — growth rate (positive = expanding outward, negative = shrinking)
    /// - `thickness` — tube radius around the spiral curve
    /// - `turns` — number of full turns
    ///
    /// # Panics
    ///
    /// Panics if `a` or `thickness` is not positive and finite, if `b` is
    /// not finite, or if `turns` is not positive and finite.
    #[must_use]
    pub fn log_spiral(a: f64, b: f64, thickness: f64, turns: f64) -> Self {
        assert!(
            a > 0.0 && a.is_finite(),
            "log_spiral initial radius a must be positive and finite, got {a}"
        );
        assert!(
            b.is_finite(),
            "log_spiral growth rate b must be finite, got {b}"
        );
        assert!(
            thickness > 0.0 && thickness.is_finite(),
            "log_spiral thickness must be positive and finite, got {thickness}"
        );
        assert!(
            turns > 0.0 && turns.is_finite(),
            "log_spiral turns must be positive and finite, got {turns}"
        );
        Self {
            node: FieldNode::LogSpiral {
                a,
                b,
                thickness,
                turns,
            },
        }
    }

    /// Gyroid triply-periodic minimal surface — lightweight lattice infill.
    ///
    /// Infinite geometry (like `Plane`). Must be intersected with a finite
    /// solid for meshing:
    /// ```ignore
    /// Solid::cuboid(half).intersect(Solid::gyroid(scale, thickness))
    /// ```
    ///
    /// - `scale` — spatial frequency (higher = denser lattice).
    ///   Period = `2π / scale` along each axis.
    /// - `thickness` — wall thickness of the lattice sheets.
    ///
    /// # Panics
    ///
    /// Panics if `scale` or `thickness` is not positive and finite.
    #[must_use]
    pub fn gyroid(scale: f64, thickness: f64) -> Self {
        assert!(
            scale > 0.0 && scale.is_finite(),
            "gyroid scale must be positive and finite, got {scale}"
        );
        assert!(
            thickness > 0.0 && thickness.is_finite(),
            "gyroid thickness must be positive and finite, got {thickness}"
        );
        Self {
            node: FieldNode::Gyroid { scale, thickness },
        }
    }

    /// Schwarz P triply-periodic minimal surface — alternative lattice infill.
    ///
    /// Infinite geometry (like `Plane`). Must be intersected with a finite
    /// solid for meshing:
    /// ```ignore
    /// Solid::cuboid(half).intersect(Solid::schwarz_p(scale, thickness))
    /// ```
    ///
    /// - `scale` — spatial frequency (higher = denser lattice).
    ///   Period = `2π / scale` along each axis.
    /// - `thickness` — wall thickness of the lattice sheets.
    ///
    /// # Panics
    ///
    /// Panics if `scale` or `thickness` is not positive and finite.
    #[must_use]
    pub fn schwarz_p(scale: f64, thickness: f64) -> Self {
        assert!(
            scale > 0.0 && scale.is_finite(),
            "schwarz_p scale must be positive and finite, got {scale}"
        );
        assert!(
            thickness > 0.0 && thickness.is_finite(),
            "schwarz_p thickness must be positive and finite, got {thickness}"
        );
        Self {
            node: FieldNode::SchwarzP { scale, thickness },
        }
    }

    /// Helix tube along the Z axis — springs, coils, DNA-like structures.
    ///
    /// The helix curve spirals from `z = 0` to `z = pitch * turns`.
    ///
    /// - `radius` — distance from Z axis to the helix center
    /// - `pitch` — vertical distance per full turn
    /// - `thickness` — tube radius around the helix curve
    /// - `turns` — number of full turns
    ///
    /// Near-exact SDF via Newton refinement (like `pipe_spline`).
    ///
    /// # Panics
    ///
    /// Panics if any parameter is not positive and finite.
    #[must_use]
    pub fn helix(radius: f64, pitch: f64, thickness: f64, turns: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "helix radius must be positive and finite, got {radius}"
        );
        assert!(
            pitch > 0.0 && pitch.is_finite(),
            "helix pitch must be positive and finite, got {pitch}"
        );
        assert!(
            thickness > 0.0 && thickness.is_finite(),
            "helix thickness must be positive and finite, got {thickness}"
        );
        assert!(
            turns > 0.0 && turns.is_finite(),
            "helix turns must be positive and finite, got {turns}"
        );
        Self {
            node: FieldNode::Helix {
                radius,
                pitch,
                thickness,
                turns,
            },
        }
    }
}
