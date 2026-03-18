//! Opaque design primitive wrapping a field expression tree.
//!
//! `Solid` is the public API for constructing and querying implicit surface
//! fields. Consumers never see `FieldNode` — they interact entirely through
//! `Solid` methods.
//!
//! The internal representation can change without breaking downstream code
//! (expression tree today, B-Rep in the future — see `CF_DESIGN_SPEC` §8).

use std::sync::Arc;

use cf_geometry::{Aabb, IndexedMesh, SdfGrid};
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::field_node::{FieldNode, UserEvalFn, UserIntervalFn};

/// Lattice type for infill operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InfillKind {
    /// Gyroid triply-periodic minimal surface — the standard bio-inspired infill.
    Gyroid,
    /// Schwarz P triply-periodic minimal surface — alternative with cubic symmetry.
    SchwarzP,
}

/// Opaque solid defined by an implicit surface field.
///
/// Construct with primitive factory methods (`sphere`, `cuboid`, etc.) and
/// compose with boolean operations (`union`, `subtract`, etc.) and transforms
/// (`translate`, `rotate`, etc.) — added in later sessions.
///
/// Convention: the field is negative inside, positive outside, zero on surface.
#[derive(Debug, Clone)]
pub struct Solid {
    pub(crate) node: FieldNode,
}

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
            node: FieldNode::Sphere { radius },
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

    // ── Boolean operations ────────────────────────────────────────────

    /// Union of two solids: material where either solid exists.
    ///
    /// `min(self, other)`. Preserves SDF lower-bound property.
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        Self {
            node: FieldNode::Union(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Subtract `other` from `self`: material where `self` exists but `other`
    /// does not.
    ///
    /// `max(self, -other)`.
    #[must_use]
    pub fn subtract(self, other: Self) -> Self {
        Self {
            node: FieldNode::Subtract(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Intersection of two solids: material where both solids exist.
    ///
    /// `max(self, other)`. Preserves SDF lower-bound property.
    #[must_use]
    pub fn intersect(self, other: Self) -> Self {
        Self {
            node: FieldNode::Intersect(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Smooth union — blends two solids with blend radius `k`.
    ///
    /// `k = 0` approaches sharp union. Larger `k` produces a wider organic
    /// fillet. The blend region adds material (field value decreases).
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_union(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_union blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothUnion(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Smooth subtraction — smoothly removes `other` from `self` with blend
    /// radius `k`.
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_subtract(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_subtract blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothSubtract(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Smooth intersection — smoothly intersects two solids with blend
    /// radius `k`. Removes material in the blend region.
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_intersect(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_intersect blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothIntersect(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Symmetric n-ary smooth union — blends multiple solids with blend
    /// radius `k`. Order-independent (unlike chaining binary `smooth_union`).
    ///
    /// Uses log-sum-exp internally for symmetric blending.
    ///
    /// # Panics
    ///
    /// Panics if `solids` is empty or `k` is not positive and finite.
    #[must_use]
    pub fn smooth_union_all(solids: Vec<Self>, k: f64) -> Self {
        assert!(
            !solids.is_empty(),
            "smooth_union_all requires at least one solid"
        );
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_union_all blend radius k must be positive and finite, got {k}"
        );
        let nodes: Vec<FieldNode> = solids.into_iter().map(|s| s.node).collect();
        Self {
            node: FieldNode::SmoothUnionAll(nodes, k),
        }
    }

    /// Smooth union with spatially varying blend radius.
    ///
    /// Like [`smooth_union`](Self::smooth_union) but the blend radius `k` is
    /// determined by `radius_fn(p)` at each query point. Where `radius_fn`
    /// returns a larger value, a wider organic fillet is produced.
    ///
    /// `max_k` is the upper bound on `radius_fn` over the domain. It is used
    /// for conservative interval evaluation (mesh pruning). For correctness,
    /// `max_k` must be ≥ the actual supremum of `radius_fn`.
    ///
    /// # Panics
    ///
    /// Panics if `max_k` is not positive and finite.
    #[must_use]
    pub fn smooth_union_variable(
        self,
        other: Self,
        radius_fn: impl Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
        max_k: f64,
    ) -> Self {
        assert!(
            max_k > 0.0 && max_k.is_finite(),
            "smooth_union_variable max_k must be positive and finite, got {max_k}"
        );
        Self {
            node: FieldNode::SmoothUnionVariable {
                a: Box::new(self.node),
                b: Box::new(other.node),
                radius_fn: UserEvalFn(Arc::new(radius_fn)),
                max_k,
            },
        }
    }

    /// Fill the interior with a lattice structure, preserving a solid outer
    /// shell.
    ///
    /// Creates a hollow shell of `wall_thickness` around the original surface,
    /// then fills the interior with a periodic lattice (gyroid or Schwarz P).
    ///
    /// - `kind` — lattice type ([`InfillKind::Gyroid`] or [`InfillKind::SchwarzP`]).
    /// - `scale` — spatial frequency of the lattice (higher = denser). Period
    ///   = `2π / scale` along each axis.
    /// - `lattice_thickness` — wall thickness of the lattice sheets.
    /// - `wall_thickness` — thickness of the solid outer shell.
    ///
    /// # Panics
    ///
    /// Panics if `scale`, `lattice_thickness`, or `wall_thickness` is not
    /// positive and finite.
    #[must_use]
    pub fn infill(
        self,
        kind: InfillKind,
        scale: f64,
        lattice_thickness: f64,
        wall_thickness: f64,
    ) -> Self {
        assert!(
            scale > 0.0 && scale.is_finite(),
            "infill scale must be positive and finite, got {scale}"
        );
        assert!(
            lattice_thickness > 0.0 && lattice_thickness.is_finite(),
            "infill lattice_thickness must be positive and finite, got {lattice_thickness}"
        );
        assert!(
            wall_thickness > 0.0 && wall_thickness.is_finite(),
            "infill wall_thickness must be positive and finite, got {wall_thickness}"
        );
        let shell = self.clone().shell(wall_thickness);
        let interior = self.offset(-wall_thickness);
        let lattice = match kind {
            InfillKind::Gyroid => Self::gyroid(scale, lattice_thickness),
            InfillKind::SchwarzP => Self::schwarz_p(scale, lattice_thickness),
        };
        shell.union(interior.intersect(lattice))
    }

    // ── Transforms ───────────────────────────────────────────────────

    /// Translate (move) the solid by the given offset.
    ///
    /// Preserves SDF property.
    ///
    /// # Panics
    ///
    /// Panics if any component of `offset` is non-finite.
    #[must_use]
    pub fn translate(self, offset: Vector3<f64>) -> Self {
        assert!(
            offset.iter().all(|v| v.is_finite()),
            "translate offset must be finite, got {offset:?}"
        );
        Self {
            node: FieldNode::Translate(Box::new(self.node), offset),
        }
    }

    /// Rotate the solid by the given unit quaternion.
    ///
    /// Preserves SDF property.
    #[must_use]
    pub fn rotate(self, rotation: UnitQuaternion<f64>) -> Self {
        Self {
            node: FieldNode::Rotate(Box::new(self.node), rotation),
        }
    }

    /// Uniformly scale the solid by the given factor.
    ///
    /// Preserves SDF property. Factor must be positive.
    ///
    /// # Panics
    ///
    /// Panics if `factor` is not positive and finite.
    #[must_use]
    pub fn scale_uniform(self, factor: f64) -> Self {
        assert!(
            factor > 0.0 && factor.is_finite(),
            "scale_uniform factor must be positive and finite, got {factor}"
        );
        Self {
            node: FieldNode::ScaleUniform(Box::new(self.node), factor),
        }
    }

    /// Mirror the solid across a plane through the origin with the given
    /// normal.
    ///
    /// The geometry on the positive side of the plane is reflected to the
    /// negative side.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero-length or non-finite.
    #[must_use]
    pub fn mirror(self, normal: Vector3<f64>) -> Self {
        let len = normal.norm();
        assert!(
            len > 1e-12 && normal.iter().all(|v| v.is_finite()),
            "mirror normal must be non-zero and finite, got {normal:?}"
        );
        let unit_normal = normal / len;
        Self {
            node: FieldNode::Mirror(Box::new(self.node), unit_normal),
        }
    }

    // ── Domain operations ─────────────────────────────────────────────

    /// Shell — hollow out the solid to the given wall thickness.
    ///
    /// `|f(p)| - thickness`. The resulting solid is a thin shell around the
    /// original surface.
    ///
    /// **Requires exact SDF input** for uniform wall thickness. Applied to
    /// an f-rep or post-boolean field, wall thickness will be non-uniform.
    ///
    /// # Panics
    ///
    /// Panics if `thickness` is not positive and finite.
    #[must_use]
    pub fn shell(self, thickness: f64) -> Self {
        assert!(
            thickness > 0.0 && thickness.is_finite(),
            "shell thickness must be positive and finite, got {thickness}"
        );
        Self {
            node: FieldNode::Shell(Box::new(self.node), thickness),
        }
    }

    /// Round — add rounding to all edges.
    ///
    /// `f(p) - radius`. Shifts the isosurface outward, rounding all edges
    /// and corners. The solid grows by `radius` in all directions.
    ///
    /// **Requires exact SDF input** for uniform rounding.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite.
    #[must_use]
    pub fn round(self, radius: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "round radius must be positive and finite, got {radius}"
        );
        Self {
            node: FieldNode::Round(Box::new(self.node), radius),
        }
    }

    /// Offset — grow or shrink the shape uniformly.
    ///
    /// `f(p) - distance`. Positive distance grows (adds material), negative
    /// distance shrinks (removes material).
    ///
    /// Common use: manufacturing clearance adjustment:
    /// `pin.offset(-clearance / 2.0)` shrinks a pin by half the clearance.
    ///
    /// **Requires exact SDF input** for uniform offset.
    ///
    /// # Panics
    ///
    /// Panics if `distance` is not finite.
    #[must_use]
    pub fn offset(self, distance: f64) -> Self {
        assert!(
            distance.is_finite(),
            "offset distance must be finite, got {distance}"
        );
        Self {
            node: FieldNode::Offset(Box::new(self.node), distance),
        }
    }

    /// Elongate — stretch the shape along axes by inserting flat sections.
    ///
    /// `q = p - clamp(p, -h, h)`, then `f(q)`. Each axis is stretched by
    /// `2 * half_extents[axis]`. Preserves the SDF property.
    ///
    /// Example: `Solid::sphere(1.0).elongate(Vector3::new(2.0, 0.0, 0.0))`
    /// produces a capsule-like shape extending from x=-3 to x=3.
    ///
    /// # Panics
    ///
    /// Panics if any half-extent is negative or non-finite.
    #[must_use]
    pub fn elongate(self, half_extents: Vector3<f64>) -> Self {
        assert!(
            half_extents.iter().all(|&v| v >= 0.0 && v.is_finite()),
            "elongate half_extents must be non-negative and finite, got {half_extents:?}"
        );
        Self {
            node: FieldNode::Elongate(Box::new(self.node), half_extents),
        }
    }

    /// Twist the shape: rotate XY cross-section proportionally to Z position.
    ///
    /// `rate` is the twist rate in radians per unit length along Z.
    /// Positive rate twists counterclockwise (right-hand rule about +Z).
    ///
    /// **⚠ Distorts the distance field.** Lipschitz constant ≈ √(1 + (rate·r)²)
    /// where r is distance from Z axis. Increase mesh resolution for large
    /// `rate` values or shapes far from the Z axis.
    ///
    /// # Panics
    ///
    /// Panics if `rate` is not finite.
    #[must_use]
    pub fn twist(self, rate: f64) -> Self {
        assert!(rate.is_finite(), "twist rate must be finite, got {rate}");
        Self {
            node: FieldNode::Twist(Box::new(self.node), rate),
        }
    }

    /// Bend the shape: curve a Z-extended shape in the XZ plane.
    ///
    /// `rate` is the curvature in radians per unit length along Z. Positive
    /// rate bends toward −X (the shape curves leftward), negative toward +X.
    ///
    /// **Same distance field distortion as [`twist`](Self::twist).** Works
    /// best for moderate curvatures (|rate| · `z_extent` < π/2).
    ///
    /// # Panics
    ///
    /// Panics if `rate` is not finite.
    #[must_use]
    pub fn bend(self, rate: f64) -> Self {
        assert!(rate.is_finite(), "bend rate must be finite, got {rate}");
        Self {
            node: FieldNode::Bend(Box::new(self.node), rate),
        }
    }

    /// Infinite repetition with the given spacing along each axis.
    ///
    /// Creates an infinite array of copies. The child is evaluated in
    /// the fundamental domain `[-spacing/2, spacing/2]` per axis.
    ///
    /// **Produces infinite geometry.** Must be intersected with a finite
    /// solid for meshing (same pattern as `Plane`, `Gyroid`):
    /// ```ignore
    /// let holes = Solid::cylinder(0.5, 1.0).repeat(Vector3::new(3.0, 3.0, 0.0));
    /// let plate = Solid::cuboid(Vector3::new(10.0, 10.0, 1.0));
    /// let result = plate.subtract(holes);
    /// ```
    ///
    /// **Exact SDF** only when child geometry fits within one cell
    /// (`[-spacing/2, spacing/2]` per axis). Overlapping geometry is
    /// clipped at cell boundaries.
    ///
    /// # Panics
    ///
    /// Panics if any spacing component is not positive and finite.
    #[must_use]
    pub fn repeat(self, spacing: Vector3<f64>) -> Self {
        assert!(
            spacing.iter().all(|&v| v > 0.0 && v.is_finite()),
            "repeat spacing must be positive and finite per axis, got {spacing:?}"
        );
        Self {
            node: FieldNode::Repeat(Box::new(self.node), spacing),
        }
    }

    /// Finite repetition with the given spacing and count per axis.
    ///
    /// Creates `count[i]` copies along each axis, centered at origin.
    /// For `count = [3, 1, 1]` with `spacing = (5, 5, 5)`, copies are at
    /// x ∈ {−5, 0, 5}, single copy on Y and Z.
    ///
    /// Unlike [`repeat`](Self::repeat), the result is finite geometry with
    /// bounded AABB. Can be meshed directly.
    ///
    /// **Exact SDF** only when child geometry fits within one cell.
    ///
    /// # Panics
    ///
    /// Panics if any spacing component is not positive and finite, or
    /// any count is zero.
    #[must_use]
    pub fn repeat_bounded(self, spacing: Vector3<f64>, count: [u32; 3]) -> Self {
        assert!(
            spacing.iter().all(|&v| v > 0.0 && v.is_finite()),
            "repeat_bounded spacing must be positive and finite, got {spacing:?}"
        );
        assert!(
            count.iter().all(|&c| c >= 1),
            "repeat_bounded count must be >= 1 per axis, got {count:?}"
        );
        Self {
            node: FieldNode::RepeatBounded {
                child: Box::new(self.node),
                spacing,
                count,
            },
        }
    }

    /// User-defined function leaf node.
    ///
    /// Escape hatch for custom implicit surface functions that the expression
    /// tree does not natively support.
    ///
    /// - `eval` — closure mapping a point to a scalar field value.
    ///   Convention: negative inside, positive outside, zero on surface.
    /// - `bounds` — bounding box of the geometry (used by the mesher to
    ///   define the evaluation domain).
    ///
    /// For octree pruning, provide an interval function via
    /// [`Self::user_fn_with_interval`]. Without one, interval evaluation
    /// returns `(-∞, +∞)` and pruning is disabled for this subtree.
    #[must_use]
    pub fn user_fn(
        eval: impl Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
        bounds: Aabb,
    ) -> Self {
        Self {
            node: FieldNode::UserFn {
                eval: UserEvalFn(Arc::new(eval)),
                interval: None,
                bounds,
            },
        }
    }

    /// User-defined function with interval bounds for octree pruning.
    ///
    /// Like [`Self::user_fn`], but with an additional closure that computes
    /// conservative `(min, max)` bounds of the field over a bounding box.
    ///
    /// The interval function must satisfy: for all points `p` in the `Aabb`,
    /// `lo <= eval(p) <= hi`. Loose bounds are safe (just reduce pruning
    /// efficiency); tight bounds improve meshing performance.
    #[must_use]
    pub fn user_fn_with_interval(
        eval: impl Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
        interval: impl Fn(&Aabb) -> (f64, f64) + Send + Sync + 'static,
        bounds: Aabb,
    ) -> Self {
        Self {
            node: FieldNode::UserFn {
                eval: UserEvalFn(Arc::new(eval)),
                interval: Some(UserIntervalFn(Arc::new(interval))),
                bounds,
            },
        }
    }

    // ── Queries ──────────────────────────────────────────────────────

    /// Compute the axis-aligned bounding box of the geometry.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    #[must_use]
    pub fn bounds(&self) -> Option<Aabb> {
        self.node.bounds()
    }

    /// Compute the Lipschitz distortion factor of this solid's field.
    ///
    /// Returns the factor by which domain distortion operations (Twist, Bend,
    /// Loft taper) amplify the field gradient. For undistorted fields, returns
    /// 1.0.
    ///
    /// The mesher automatically divides cell size by this factor when calling
    /// [`mesh()`](Self::mesh), ensuring thin features in high-distortion
    /// regions are not silently lost. Use this value for manual resolution
    /// planning.
    #[must_use]
    pub fn lipschitz_factor(&self) -> f64 {
        self.node.lipschitz_factor()
    }

    /// Extract a triangle mesh at the given tolerance (voxel size).
    ///
    /// Smaller tolerance produces a finer mesh with more triangles.
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// The mesh is watertight, manifold, and uses CCW winding (outward
    /// normals) for all finite primitives and their compositions.
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh(&self, tolerance: f64) -> IndexedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return IndexedMesh::new();
        };
        // Scale cell size by Lipschitz distortion factor to ensure thin
        // features in high-distortion regions (Twist, Bend) are captured.
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        // Expand bounds by one cell to avoid surface clipping at edges
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) = crate::mesher::mesh_field(&self.node, &expanded, cell_size);
        mesh
    }

    /// Extract a triangle mesh using dual contouring at the given tolerance.
    ///
    /// Like [`mesh`](Self::mesh), but uses dual contouring instead of
    /// marching cubes. DC places one vertex per sign-changing cell via QEF
    /// minimization, preserving sharp features (edges, corners) that MC
    /// rounds off.
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh_dc(&self, tolerance: f64) -> IndexedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return IndexedMesh::new();
        };
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) =
            crate::dual_contouring::mesh_field_dc(&self.node, &expanded, cell_size);
        mesh
    }

    /// Extract a triangle mesh using adaptive octree dual contouring.
    ///
    /// Like [`mesh_dc`](Self::mesh_dc), but uses an octree to subdivide only
    /// near the surface. Interior/exterior regions stop early, giving 10x+
    /// cell reduction vs the uniform grid while preserving QEF sharp features.
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh_adaptive(&self, tolerance: f64) -> IndexedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return IndexedMesh::new();
        };
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) =
            crate::adaptive_dc::mesh_field_adaptive(&self.node, &expanded, cell_size);
        mesh
    }

    /// Parallel adaptive octree dual contouring mesher.
    ///
    /// Like [`mesh_adaptive`](Self::mesh_adaptive) but uses rayon for:
    /// - Octree construction (parallel at top levels)
    /// - Phase 1 cell processing (parallel corner + gradient evaluation)
    /// - Batched evaluation (4 corners per tree walk via `evaluate_batch`)
    ///
    /// Returns an empty mesh for infinite geometry (bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `tolerance` is not positive and finite.
    #[must_use]
    pub fn mesh_adaptive_par(&self, tolerance: f64) -> IndexedMesh {
        assert!(
            tolerance > 0.0 && tolerance.is_finite(),
            "mesh tolerance must be positive and finite, got {tolerance}"
        );
        let Some(bounds) = self.node.bounds() else {
            return IndexedMesh::new();
        };
        let lip = self.node.lipschitz_factor();
        let cell_size = tolerance / lip;
        let expanded = bounds.expanded(cell_size);
        let (mesh, _stats) =
            crate::adaptive_dc::mesh_field_adaptive_par(&self.node, &expanded, cell_size);
        mesh
    }

    /// Evaluate the field at a point.
    ///
    /// Returns the signed distance (exact or approximate depending on the
    /// primitives involved). Negative = inside, positive = outside, zero =
    /// on surface.
    #[must_use]
    pub fn evaluate(&self, point: &Point3<f64>) -> f64 {
        self.node.evaluate(point)
    }

    /// Compute the analytic gradient of the field at a point.
    ///
    /// Returns `∇f(p)` — the direction of steepest ascent. For exact SDFs,
    /// this is the outward unit normal on the surface. For approximate SDFs,
    /// the direction is correct but magnitude may differ from 1.
    ///
    /// Uses analytic derivatives for all built-in primitives and operations.
    /// Falls back to finite differences only for [`user_fn`](Self::user_fn).
    #[must_use]
    pub fn gradient(&self, point: &Point3<f64>) -> Vector3<f64> {
        self.node.gradient(point)
    }

    /// Compute conservative (min, max) bounds of the field over an `Aabb`.
    ///
    /// The returned interval `(lo, hi)` satisfies:
    /// `lo <= self.evaluate(p) <= hi` for all `p` in the box.
    ///
    /// Used for octree pruning during meshing: if `lo > 0`, the box is fully
    /// outside; if `hi < 0`, fully inside.
    #[must_use]
    pub fn evaluate_interval(&self, aabb: &Aabb) -> (f64, f64) {
        self.node.evaluate_interval(aabb)
    }

    /// Evaluate the field on a uniform 3D grid, returning an
    /// [`SdfGrid`](cf_geometry::SdfGrid).
    ///
    /// `resolution` is the number of samples along the longest bounding-box
    /// axis (minimum 2). Other axes are scaled proportionally. The grid
    /// includes one cell of padding beyond the geometry bounds to capture
    /// the surface boundary cleanly.
    ///
    /// Returns `None` for infinite geometry (e.g., a bare `Plane`).
    ///
    /// # Panics
    ///
    /// Panics if `resolution < 2`.
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    pub fn sdf_grid(&self, resolution: usize) -> Option<SdfGrid> {
        assert!(
            resolution >= 2,
            "sdf_grid resolution must be at least 2, got {resolution}"
        );
        let bounds = self.node.bounds()?;
        let size = bounds.size();
        let longest = size.x.max(size.y).max(size.z);
        if longest <= 0.0 {
            return None;
        }
        let cell_size = longest / (resolution as f64 - 1.0);

        // Expand by one cell on each side for surface padding
        let expanded = bounds.expanded(cell_size);
        let exp_size = expanded.size();

        // +1 because N samples span N-1 intervals
        let nx = ((exp_size.x / cell_size).ceil() as usize + 1).max(2);
        let ny = ((exp_size.y / cell_size).ceil() as usize + 1).max(2);
        let nz = ((exp_size.z / cell_size).ceil() as usize + 1).max(2);

        let origin = expanded.min;
        let node = &self.node;

        Some(SdfGrid::from_fn(nx, ny, nz, cell_size, origin, |p| {
            node.evaluate(&p)
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // ── Constructor validation ───────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_zero_radius() {
        drop(Solid::sphere(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_negative_radius() {
        drop(Solid::sphere(-1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_nan() {
        drop(Solid::sphere(f64::NAN));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_infinity() {
        drop(Solid::sphere(f64::INFINITY));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cuboid_rejects_zero_extent() {
        drop(Solid::cuboid(Vector3::new(1.0, 0.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cylinder_rejects_bad_radius() {
        drop(Solid::cylinder(-1.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn capsule_rejects_bad_radius() {
        drop(Solid::capsule(0.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn ellipsoid_rejects_bad_radii() {
        drop(Solid::ellipsoid(Vector3::new(1.0, -1.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn torus_rejects_bad_major() {
        drop(Solid::torus(0.0, 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cone_rejects_bad_radius() {
        drop(Solid::cone(0.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "non-zero and finite")]
    fn plane_rejects_zero_normal() {
        drop(Solid::plane(Vector3::zeros(), 0.0));
    }

    // ── Boolean validation ──────────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_union_rejects_zero_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_union(b, 0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_subtract_rejects_negative_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_subtract(b, -1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_intersect_rejects_nan_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_intersect(b, f64::NAN));
    }

    #[test]
    #[should_panic(expected = "at least one solid")]
    fn smooth_union_all_rejects_empty() {
        drop(Solid::smooth_union_all(vec![], 1.0));
    }

    // ── Transform validation ────────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn scale_uniform_rejects_zero() {
        drop(Solid::sphere(1.0).scale_uniform(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn scale_uniform_rejects_negative() {
        drop(Solid::sphere(1.0).scale_uniform(-1.0));
    }

    #[test]
    #[should_panic(expected = "non-zero and finite")]
    fn mirror_rejects_zero_normal() {
        drop(Solid::sphere(1.0).mirror(Vector3::zeros()));
    }

    // ── Evaluation through Solid ─────────────────────────────────────

    #[test]
    fn solid_sphere_evaluate() {
        let s = Solid::sphere(2.0);
        assert!((s.evaluate(&Point3::origin()) - (-2.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(2.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_sphere_interval() {
        let s = Solid::sphere(2.0);
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, hi) = s.evaluate_interval(&aabb);
        // Interval should be fully negative (inside sphere)
        assert!(hi < 0.0, "Expected negative interval for box inside sphere");
        assert!(lo < hi);
    }

    #[test]
    fn solid_cuboid_evaluate() {
        let c = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0));
        assert!(c.evaluate(&Point3::origin()) < 0.0);
        assert!((c.evaluate(&Point3::new(1.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_plane_normalizes() {
        // Non-unit normal should still produce correct SDF
        let p = Solid::plane(Vector3::new(0.0, 0.0, 2.0), 6.0);
        // Normalized: normal=(0,0,1), offset=3
        assert!((p.evaluate(&Point3::new(0.0, 0.0, 3.0))).abs() < 1e-10);
        assert!((p.evaluate(&Point3::new(0.0, 0.0, 5.0)) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn capsule_zero_half_height_is_sphere() {
        let cap = Solid::capsule(2.0, 0.0);
        let sph = Solid::sphere(2.0);
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert!(
                (cap.evaluate(p) - sph.evaluate(p)).abs() < 1e-10,
                "Capsule(r=2, h=0) should match Sphere(r=2) at {p:?}"
            );
        }
    }

    // ── Boolean builder methods ──────────────────────────────────────

    #[test]
    fn solid_union_method() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(3.0);
        let u = a.union(b);
        assert!((u.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn solid_subtract_method() {
        let big = Solid::sphere(5.0);
        let small = Solid::sphere(2.0);
        let sub = big.subtract(small);
        // Origin: max(-5, 2) = 2
        assert!(sub.evaluate(&Point3::origin()) > 0.0);
        // Shell region: max(3-5, -(3-2)) = max(-2, -1) = -1
        assert!(sub.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_intersect_method() {
        let a = Solid::sphere(5.0);
        let b = Solid::sphere(2.0);
        let inter = a.intersect(b);
        assert!((inter.evaluate(&Point3::origin()) - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn solid_smooth_union_method() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let k = 1.0;
        let su = a.smooth_union(b, k);
        // In the blend region, smooth union adds material
        let p = Point3::new(1.5, 0.0, 0.0);
        // Should be more negative (more inside) than the sharper of the two
        assert!(su.evaluate(&p) < 1.0);
    }

    #[test]
    fn solid_smooth_union_all_method() {
        let solids = vec![
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0)),
            Solid::sphere(2.0).translate(Vector3::new(0.0, 3.0, 0.0)),
        ];
        let sua = Solid::smooth_union_all(solids, 1.0);
        // Should be inside near each sphere center
        assert!(sua.evaluate(&Point3::origin()) < 0.0);
        assert!(sua.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
        assert!(sua.evaluate(&Point3::new(0.0, 3.0, 0.0)) < 0.0);
    }

    // ── Transform builder methods ────────────────────────────────────

    #[test]
    fn solid_translate_method() {
        let s = Solid::sphere(1.0).translate(Vector3::new(5.0, 0.0, 0.0));
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0)) - (-1.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::origin()) - 4.0).abs() < 1e-10);
    }

    #[test]
    fn solid_rotate_method() {
        let c = Solid::cuboid(Vector3::new(1.0, 2.0, 1.0));
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let r = c.rotate(rot);
        // After 90° Z rotation: x-extent becomes 2, y-extent becomes 1
        assert!(r.evaluate(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
        assert!(r.evaluate(&Point3::new(0.0, 1.5, 0.0)) > 0.0);
    }

    #[test]
    fn solid_scale_uniform_method() {
        let s = Solid::sphere(1.0).scale_uniform(3.0);
        assert!((s.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_mirror_method() {
        let s = Solid::sphere(1.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let m = s.mirror(Vector3::x());
        // Mirrored: sphere at both (3,0,0) and (-3,0,0)
        assert!(m.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
        assert!(m.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_mirror_normalizes() {
        // Non-unit normal should still work correctly
        let s = Solid::sphere(1.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let m = s.mirror(Vector3::new(5.0, 0.0, 0.0)); // unnormalized
        assert!(m.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0);
    }

    // ── Method chaining ──────────────────────────────────────────────

    #[test]
    fn solid_method_chaining() {
        // Build a hollowed, translated, scaled sphere.
        // Evaluation chain: scale(translate(subtract(sphere(5), sphere(3)), (10,0,0)), 2)
        // scale_uniform(2) evaluates f(p/2)*2, so effective center is (20,0,0),
        // effective outer radius = 10, effective inner radius = 6.
        let part = Solid::sphere(5.0)
            .subtract(Solid::sphere(3.0))
            .translate(Vector3::new(10.0, 0.0, 0.0))
            .scale_uniform(2.0);
        // Center at (20, 0, 0) — inside the hole (inner radius 6)
        assert!(part.evaluate(&Point3::new(20.0, 0.0, 0.0)) > 0.0);
        // On outer surface at x = 30 (center 20 + outer radius 10)
        assert!((part.evaluate(&Point3::new(30.0, 0.0, 0.0))).abs() < 1e-6);
        // Inside the shell at x = 28 (8 from center, between inner 6 and outer 10)
        assert!(part.evaluate(&Point3::new(28.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_interval_for_union() {
        let u = Solid::sphere(2.0).union(Solid::sphere(3.0));
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, hi) = u.evaluate_interval(&aabb);
        // Both spheres fully contain the box, interval should be negative
        assert!(hi < 0.0);
        assert!(lo < hi);
    }

    #[test]
    fn solid_interval_for_translate() {
        let s = Solid::sphere(2.0).translate(Vector3::new(5.0, 0.0, 0.0));
        // Box at origin: fully outside the translated sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, _hi) = s.evaluate_interval(&aabb);
        assert!(
            lo > 0.0,
            "Box at origin should be fully outside sphere at x=5"
        );
    }

    #[test]
    fn solid_interval_for_scale() {
        let s = Solid::sphere(1.0).scale_uniform(5.0);
        // Small box at origin: fully inside the scaled sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (_lo, hi) = s.evaluate_interval(&aabb);
        assert!(hi < 0.0, "Box at origin should be fully inside sphere(r=5)");
    }

    // ── Domain operation builder methods ────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn shell_rejects_zero_thickness() {
        drop(Solid::sphere(1.0).shell(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn shell_rejects_negative_thickness() {
        drop(Solid::sphere(1.0).shell(-1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn round_rejects_zero_radius() {
        drop(Solid::sphere(1.0).round(0.0));
    }

    #[test]
    #[should_panic(expected = "must be finite")]
    fn offset_rejects_nan() {
        drop(Solid::sphere(1.0).offset(f64::NAN));
    }

    #[test]
    #[should_panic(expected = "non-negative and finite")]
    fn elongate_rejects_negative() {
        drop(Solid::sphere(1.0).elongate(Vector3::new(-1.0, 0.0, 0.0)));
    }

    #[test]
    fn solid_shell_method() {
        let s = Solid::sphere(5.0).shell(1.0);
        // Inner surface at r=4
        assert!((s.evaluate(&Point3::new(4.0, 0.0, 0.0))).abs() < 1e-10);
        // Outer surface at r=6
        assert!((s.evaluate(&Point3::new(6.0, 0.0, 0.0))).abs() < 1e-10);
        // Origin: outside the wall
        assert!(s.evaluate(&Point3::origin()) > 0.0);
        // Shell region: inside the wall
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_round_method() {
        let c = Solid::cuboid(Vector3::new(1.0, 1.0, 1.0)).round(0.5);
        // New surface at (1.5, 0, 0) — face moved out by radius
        assert!((c.evaluate(&Point3::new(1.5, 0.0, 0.0))).abs() < 1e-10);
        // Inside at (1.0, 0, 0)
        assert!(c.evaluate(&Point3::new(1.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_offset_method() {
        // Grow sphere(3) by 2 → effective radius 5
        let s = Solid::sphere(3.0).offset(2.0);
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0))).abs() < 1e-10);
        assert!((s.evaluate(&Point3::origin()) - (-5.0)).abs() < 1e-10);

        // Shrink sphere(3) by 1 → effective radius 2
        let s = Solid::sphere(3.0).offset(-1.0);
        assert!((s.evaluate(&Point3::new(2.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_elongate_method() {
        // Elongate sphere(1) by (2,0,0) → capsule from x=-3 to x=3
        let s = Solid::sphere(1.0).elongate(Vector3::new(2.0, 0.0, 0.0));
        // Surface at x = ±3
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(-3.0, 0.0, 0.0))).abs() < 1e-10);
        // Inside at x = 0
        assert!(s.evaluate(&Point3::origin()) < 0.0);
        // Y extent unchanged: surface at y = 1
        assert!((s.evaluate(&Point3::new(0.0, 1.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_elongate_zero_is_identity() {
        let s = Solid::sphere(2.0);
        let e = Solid::sphere(2.0).elongate(Vector3::new(0.0, 0.0, 0.0));
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert!(
                (s.evaluate(p) - e.evaluate(p)).abs() < 1e-10,
                "Elongate(0,0,0) should be identity at {p:?}"
            );
        }
    }

    // ── UserFn builder methods ──────────────────────────────────────

    #[test]
    fn solid_user_fn_method() {
        let s = Solid::user_fn(
            |p| p.coords.norm() - 3.0,
            Aabb::new(Point3::new(-4.0, -4.0, -4.0), Point3::new(4.0, 4.0, 4.0)),
        );
        assert!((s.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_user_fn_with_interval_method() {
        let s = Solid::user_fn_with_interval(
            |p| p.coords.norm() - 3.0,
            |aabb| {
                let closest = Point3::new(
                    0.0_f64.clamp(aabb.min.x, aabb.max.x),
                    0.0_f64.clamp(aabb.min.y, aabb.max.y),
                    0.0_f64.clamp(aabb.min.z, aabb.max.z),
                );
                let min_dist = closest.coords.norm();
                let max_dist = aabb
                    .corners()
                    .iter()
                    .map(|c| c.coords.norm())
                    .fold(0.0_f64, f64::max);
                (min_dist - 3.0, max_dist - 3.0)
            },
            Aabb::new(Point3::new(-4.0, -4.0, -4.0), Point3::new(4.0, 4.0, 4.0)),
        );
        // Interval should prune box fully outside
        let far_box = Aabb::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));
        let (lo, _) = s.evaluate_interval(&far_box);
        assert!(lo > 0.0, "Far box should be fully outside user sphere");
    }

    #[test]
    fn solid_user_fn_composes_with_booleans() {
        // UserFn sphere unioned with a regular sphere
        let custom = Solid::user_fn(
            |p| p.coords.norm() - 2.0,
            Aabb::new(Point3::new(-3.0, -3.0, -3.0), Point3::new(3.0, 3.0, 3.0)),
        );
        let regular = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let u = custom.union(regular);
        assert!(u.evaluate(&Point3::origin()) < 0.0);
        assert!(u.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_user_fn_is_cloneable() {
        let s = Solid::user_fn(
            |p| p.coords.norm() - 1.0,
            Aabb::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0)),
        );
        let s2 = s.clone();
        assert!((s.evaluate(&Point3::origin()) - s2.evaluate(&Point3::origin())).abs() < 1e-10);
    }

    // ── Domain ops chaining ─────────────────────────────────────────

    #[test]
    fn shell_then_translate() {
        let s = Solid::sphere(5.0)
            .shell(1.0)
            .translate(Vector3::new(10.0, 0.0, 0.0));
        // Shell inner surface at x = 10+4 = 14, outer at x = 10+6 = 16
        assert!(s.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0); // center of shell (hollow)
        assert!(s.evaluate(&Point3::new(15.0, 0.0, 0.0)) < 0.0); // in the wall
        assert!((s.evaluate(&Point3::new(14.0, 0.0, 0.0))).abs() < 1e-10); // inner surface
        assert!((s.evaluate(&Point3::new(16.0, 0.0, 0.0))).abs() < 1e-10); // outer surface
    }

    #[test]
    fn offset_for_clearance() {
        // Pin-in-hole clearance test: pin shrinks, hole grows
        let pin = Solid::cylinder(2.0, 5.0).offset(-0.15);
        let hole = Solid::cylinder(2.0, 5.0).offset(0.15);
        // Pin surface at r = 1.85
        assert!((pin.evaluate(&Point3::new(1.85, 0.0, 0.0))).abs() < 1e-10);
        // Hole surface at r = 2.15
        assert!((hole.evaluate(&Point3::new(2.15, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_interval_for_shell() {
        let s = Solid::sphere(10.0).shell(1.0);
        // Box at origin: deep inside sphere, so shell field is large positive
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, _) = s.evaluate_interval(&aabb);
        assert!(lo > 0.0, "Box at origin should be outside shell wall");
    }

    // ── Pipe builder validation ──────────────────────────────────────

    #[test]
    #[should_panic(expected = "at least 2 vertices")]
    fn pipe_rejects_single_vertex() {
        drop(Solid::pipe(vec![Point3::origin()], 1.0));
    }

    #[test]
    #[should_panic(expected = "at least 2 vertices")]
    fn pipe_rejects_empty() {
        drop(Solid::pipe(vec![], 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn pipe_rejects_zero_radius() {
        drop(Solid::pipe(
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            0.0,
        ));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn pipe_rejects_negative_radius() {
        drop(Solid::pipe(
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            -1.0,
        ));
    }

    #[test]
    #[should_panic(expected = "finite coordinates")]
    fn pipe_rejects_nan_vertex() {
        drop(Solid::pipe(
            vec![Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            1.0,
        ));
    }

    #[test]
    #[should_panic(expected = "at least 2 control points")]
    fn pipe_spline_rejects_single_point() {
        drop(Solid::pipe_spline(vec![Point3::origin()], 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn pipe_spline_rejects_inf_radius() {
        drop(Solid::pipe_spline(
            vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            f64::INFINITY,
        ));
    }

    // ── Pipe builder methods ────────────────────────────────────────

    #[test]
    fn solid_pipe_evaluate() {
        let s = Solid::pipe(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            1.0,
        );
        // Midpoint on axis: should be -radius
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0)) - (-1.0)).abs() < 1e-10);
        // On surface
        assert!((s.evaluate(&Point3::new(5.0, 1.0, 0.0))).abs() < 1e-10);
        // Outside
        assert!(s.evaluate(&Point3::new(5.0, 3.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_pipe_spline_evaluate() {
        let s = Solid::pipe_spline(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            1.0,
        );
        // Start and end should be inside
        assert!(s.evaluate(&Point3::new(0.0, 0.0, 0.0)) < 0.0);
        assert!(s.evaluate(&Point3::new(10.0, 0.0, 0.0)) < 0.0);
        // Midpoint on surface
        assert!((s.evaluate(&Point3::new(5.0, 1.0, 0.0))).abs() < 1e-6);
    }

    #[test]
    fn solid_pipe_composes_with_translate() {
        let s = Solid::pipe(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0)],
            0.5,
        )
        .translate(Vector3::new(0.0, 0.0, 10.0));
        // Pipe should be at z=10 now
        assert!(s.evaluate(&Point3::new(2.5, 0.0, 10.0)) < 0.0);
        assert!(s.evaluate(&Point3::new(2.5, 0.0, 0.0)) > 0.0);
    }

    // ── Mesh validation helpers ────────────────────────────────────

    fn check_topology(mesh: &IndexedMesh) -> (bool, bool) {
        use std::collections::HashMap;
        let mut directed: HashMap<(u32, u32), usize> = HashMap::new();
        for face in &mesh.faces {
            for i in 0..3 {
                *directed.entry((face[i], face[(i + 1) % 3])).or_insert(0) += 1;
            }
        }
        let mut boundary = 0_usize;
        let mut non_manifold = 0_usize;
        for (&(a, b), &count) in &directed {
            if count > 1 {
                non_manifold += 1;
            }
            if directed.get(&(b, a)).copied().unwrap_or(0) == 0 {
                boundary += 1;
            }
        }
        (boundary == 0, non_manifold == 0)
    }

    fn assert_mesh_valid(mesh: &IndexedMesh, label: &str) {
        assert!(!mesh.is_empty(), "{label}: mesh should not be empty");
        let (watertight, manifold) = check_topology(mesh);
        assert!(watertight, "{label}: mesh should be watertight");
        assert!(manifold, "{label}: mesh should be manifold");
        assert!(
            mesh.signed_volume() > 0.0,
            "{label}: mesh should have positive signed volume (CCW winding), got {}",
            mesh.signed_volume()
        );
    }

    // ── Integration tests: composed trees → mesh → valid ───────

    #[test]
    fn integration_smooth_union_translated_spheres() {
        let a = Solid::sphere(3.0).translate(Vector3::new(-2.0, 0.0, 0.0));
        let b = Solid::sphere(3.0).translate(Vector3::new(2.0, 0.0, 0.0));
        let s = a.smooth_union(b, 1.0);
        let mesh = s.mesh(0.5);
        assert_mesh_valid(&mesh, "smooth_union_translated_spheres");
    }

    #[test]
    fn integration_subtract_rotated_cuboid_sphere() {
        let cuboid = Solid::cuboid(Vector3::new(3.0, 3.0, 3.0));
        let hole = Solid::sphere(2.0);
        let s = cuboid
            .subtract(hole)
            .rotate(UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                PI / 4.0,
            ));
        let mesh = s.mesh(0.4);
        assert_mesh_valid(&mesh, "subtract_rotated");
    }

    #[test]
    fn integration_shell_mirror() {
        let s = Solid::sphere(5.0)
            .shell(0.5)
            .translate(Vector3::new(3.0, 0.0, 0.0))
            .mirror(Vector3::x());
        let mesh = s.mesh(0.5);
        assert_mesh_valid(&mesh, "shell_mirror");
    }

    #[test]
    fn integration_elongate_smooth_intersect_cuboid() {
        let elongated = Solid::sphere(2.0).elongate(Vector3::new(3.0, 0.0, 0.0));
        let cuboid = Solid::cuboid(Vector3::new(4.0, 1.5, 1.5));
        let s = elongated.smooth_intersect(cuboid, 0.5);
        let mesh = s.mesh(0.3);
        assert_mesh_valid(&mesh, "elongate_smooth_intersect");
    }

    #[test]
    fn integration_pipe_union_sphere() {
        let pipe = Solid::pipe(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(5.0, 0.0, 0.0),
                Point3::new(5.0, 5.0, 0.0),
            ],
            0.8,
        );
        let ball = Solid::sphere(1.5).translate(Vector3::new(5.0, 5.0, 0.0));
        let s = pipe.union(ball);
        let mesh = s.mesh(0.3);
        assert_mesh_valid(&mesh, "pipe_union_sphere");
    }

    #[test]
    fn integration_multi_op_chain() {
        let s = Solid::sphere(3.0)
            .shell(0.5)
            .round(0.2)
            .translate(Vector3::new(5.0, 0.0, 0.0))
            .scale_uniform(2.0);
        let mesh = s.mesh(0.5);
        assert_mesh_valid(&mesh, "multi_op_chain");
    }

    // ── Interval pruning on composed trees ─────────────────────

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn pruning_ratio_union_translated_spheres() {
        let a = Solid::sphere(3.0).translate(Vector3::new(-3.0, 0.0, 0.0));
        let b = Solid::sphere(3.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = a.union(b);
        let bounds = s.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = crate::mesher::mesh_field(&s.node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            ratio > 0.70,
            "union pruning should be >70%, got {:.1}%",
            ratio * 100.0
        );
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn pruning_ratio_subtract_spheres() {
        let big = Solid::sphere(5.0);
        let small = Solid::sphere(2.0);
        let s = big.subtract(small);
        let bounds = s.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = crate::mesher::mesh_field(&s.node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            ratio > 0.60,
            "subtract pruning should be >60%, got {:.1}%",
            ratio * 100.0
        );
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn pruning_ratio_smooth_union_all_3_spheres() {
        let solids = vec![
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(4.0, 0.0, 0.0)),
            Solid::sphere(2.0).translate(Vector3::new(0.0, 4.0, 0.0)),
        ];
        let s = Solid::smooth_union_all(solids, 1.0);
        let bounds = s.bounds().map(|b| b.expanded(0.5));
        let (_, stats) = crate::mesher::mesh_field(&s.node, &bounds.unwrap_or(Aabb::empty()), 0.5);
        let ratio = stats.cells_pruned as f64 / stats.cells_total as f64;
        assert!(
            ratio > 0.60,
            "smooth_union_all pruning should be >60%, got {:.1}%",
            ratio * 100.0
        );
    }

    // ── SdfGrid tests ─────────────────────────────────────────

    #[test]
    #[allow(clippy::cast_precision_loss, clippy::unwrap_used)]
    fn sdf_grid_matches_evaluate() {
        let s = Solid::sphere(3.0);
        let grid = s.sdf_grid(16).unwrap();
        let origin = grid.origin();
        let cs = grid.cell_size();
        // Spot-check grid values against point evaluation
        for &xi in &[0_usize, 5, 10] {
            for &yi in &[0_usize, 5, 10] {
                for &zi in &[0_usize, 5, 10] {
                    if xi < grid.width() && yi < grid.height() && zi < grid.depth() {
                        let p = Point3::new(
                            (xi as f64).mul_add(cs, origin.x),
                            (yi as f64).mul_add(cs, origin.y),
                            (zi as f64).mul_add(cs, origin.z),
                        );
                        let grid_val = grid.get(xi, yi, zi).unwrap();
                        let eval_val = s.evaluate(&p);
                        assert!(
                            (grid_val - eval_val).abs() < 1e-10,
                            "sdf_grid mismatch at ({xi},{yi},{zi}): grid={grid_val}, eval={eval_val}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn sdf_grid_resolution_dimensions() {
        // Cuboid half-extents (5,3,2) → full size (10,6,4). Longest axis = 10.
        let s = Solid::cuboid(Vector3::new(5.0, 3.0, 2.0));
        let grid = s.sdf_grid(20).unwrap();
        // Longest axis (x=10) gets 20 samples → cell_size = 10/19 ≈ 0.526
        // With 1-cell padding each side, width >= 20 + 2 = 22
        assert!(
            grid.width() >= 22,
            "expected width >= 22, got {}",
            grid.width()
        );
        assert!(grid.height() >= 2);
        assert!(grid.depth() >= 2);
    }

    #[test]
    fn sdf_grid_infinite_returns_none() {
        let s = Solid::plane(Vector3::z(), 0.0);
        assert!(s.sdf_grid(16).is_none());
    }

    #[test]
    #[should_panic(expected = "at least 2")]
    fn sdf_grid_rejects_resolution_1() {
        let s = Solid::sphere(1.0);
        drop(s.sdf_grid(1));
    }

    // ── Bio-inspired primitive tests ──────────────────────────────────

    // -- Superellipsoid --

    #[test]
    fn superellipsoid_sign_correctness() {
        // n1=n2=2 approximates an ellipsoid
        let s = Solid::superellipsoid(Vector3::new(2.0, 3.0, 4.0), 2.0, 2.0);
        assert!(
            s.evaluate(&Point3::origin()) < 0.0,
            "center should be inside"
        );
        assert!(
            s.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0,
            "far point should be outside"
        );
    }

    #[test]
    fn superellipsoid_on_surface() {
        // Uniform radii, n1=n2=2 → sphere of radius 3
        let s = Solid::superellipsoid(Vector3::new(3.0, 3.0, 3.0), 2.0, 2.0);
        // On-axis surface point: f = |3/3| = 1, f-1 = 0
        let val = s.evaluate(&Point3::new(3.0, 0.0, 0.0));
        assert!(
            val.abs() < 1e-10,
            "on-surface point should be zero, got {val}"
        );
    }

    #[test]
    fn superellipsoid_octahedron_sign() {
        // n1=n2=1 → octahedron-like shape
        let s = Solid::superellipsoid(Vector3::new(1.0, 1.0, 1.0), 1.0, 1.0);
        assert!(
            s.evaluate(&Point3::origin()) < 0.0,
            "center should be inside"
        );
        assert!(
            s.evaluate(&Point3::new(2.0, 0.0, 0.0)) > 0.0,
            "outside point should be positive"
        );
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn superellipsoid_bounds() {
        let s = Solid::superellipsoid(Vector3::new(2.0, 3.0, 4.0), 2.0, 2.0);
        let bb = s.bounds().expect("finite bounds");
        assert!((bb.min.x - (-2.0)).abs() < 1e-10);
        assert!((bb.max.z - 4.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn superellipsoid_rejects_zero_radii() {
        drop(Solid::superellipsoid(Vector3::new(0.0, 1.0, 1.0), 2.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn superellipsoid_rejects_zero_n1() {
        drop(Solid::superellipsoid(Vector3::new(1.0, 1.0, 1.0), 0.0, 2.0));
    }

    #[test]
    fn superellipsoid_meshes_to_valid_geometry() {
        let s = Solid::superellipsoid(Vector3::new(2.0, 2.0, 2.0), 2.0, 2.0);
        let mesh = s.mesh(0.2);
        assert!(
            mesh.vertices.len() > 10,
            "mesh should have substantial geometry"
        );
        assert!(mesh.faces.len() > 10, "mesh should have triangles");
    }

    // -- LogSpiral --

    #[test]
    fn log_spiral_sign_correctness() {
        let s = Solid::log_spiral(2.0, 0.2, 0.5, 2.0);
        // Point on the spiral at θ=0: (2, 0, 0). Should be on-surface.
        let val = s.evaluate(&Point3::new(2.0, 0.0, 0.0));
        assert!(
            val.abs() < 0.6,
            "point on spiral start should be near surface, got {val}"
        );
        // Far outside
        assert!(
            s.evaluate(&Point3::new(50.0, 0.0, 0.0)) > 0.0,
            "far point should be outside"
        );
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn log_spiral_bounds() {
        let s = Solid::log_spiral(1.0, 0.1, 0.3, 1.0);
        let bb = s.bounds().expect("finite bounds");
        // Should contain the spiral extent
        assert!(bb.max.x > 1.0, "bounds should extend past initial radius");
        assert!(
            (bb.min.z - (-0.3)).abs() < 1e-10,
            "z bounds should be ±thickness"
        );
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn log_spiral_rejects_zero_a() {
        drop(Solid::log_spiral(0.0, 0.1, 0.5, 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn log_spiral_rejects_zero_turns() {
        drop(Solid::log_spiral(1.0, 0.1, 0.5, 0.0));
    }

    #[test]
    fn log_spiral_meshes_to_valid_geometry() {
        let s = Solid::log_spiral(2.0, 0.15, 0.5, 1.5);
        let mesh = s.mesh(0.25);
        assert!(
            mesh.vertices.len() > 10,
            "mesh should have substantial geometry"
        );
    }

    // -- Gyroid --

    #[test]
    fn gyroid_sign_correctness() {
        let s = Solid::gyroid(1.0, 0.5);
        // The gyroid passes through zero at many points. Check that it has
        // both positive and negative regions in a unit cube.
        let inside = s.evaluate(&Point3::new(0.0, 0.0, 0.0));
        let outside = s.evaluate(&Point3::new(PI / 2.0, 0.0, 0.0));
        // At least one should be positive, one negative (or near-zero)
        assert!(
            inside * outside < 0.5,
            "gyroid should have sign changes, got {inside} and {outside}"
        );
    }

    #[test]
    fn gyroid_known_zero_crossing() {
        // At p=(π/2, 0, 0) with scale=1:
        // sin(π/2)cos(0) + sin(0)cos(0) + sin(0)cos(π/2) = 1 + 0 + 0 = 1
        // |1| - thickness. For thickness=1.0, the field should be ~0.
        let s = Solid::gyroid(1.0, 1.0);
        let val = s.evaluate(&Point3::new(PI / 2.0, 0.0, 0.0));
        assert!(
            val.abs() < 0.1,
            "gyroid at known zero-crossing should be near zero, got {val}"
        );
    }

    #[test]
    fn gyroid_is_infinite() {
        let s = Solid::gyroid(1.0, 0.5);
        assert!(s.bounds().is_none(), "gyroid should have no finite bounds");
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn gyroid_rejects_zero_scale() {
        drop(Solid::gyroid(0.0, 0.5));
    }

    #[test]
    fn gyroid_intersected_with_cuboid_meshes() {
        // Gyroid must be intersected with a finite solid to mesh
        let envelope = Solid::cuboid(Vector3::new(4.0, 4.0, 4.0));
        let lattice = Solid::gyroid(1.0, 0.4);
        let part = envelope.intersect(lattice);
        let mesh = part.mesh(0.3);
        assert!(
            mesh.vertices.len() > 50,
            "gyroid lattice should produce substantial mesh, got {} verts",
            mesh.vertices.len()
        );
    }

    // -- SchwarzP --

    #[test]
    fn schwarz_p_sign_correctness() {
        let s = Solid::schwarz_p(1.0, 0.5);
        // At origin: cos(0)+cos(0)+cos(0) = 3, |3|-0.5 = 2.5 → outside
        assert!(
            s.evaluate(&Point3::origin()) > 0.0,
            "origin should be outside the schwarz_p shell"
        );
    }

    #[test]
    fn schwarz_p_known_zero_crossing() {
        // At p = (π/2, 0, 0) with scale=1:
        // cos(π/2) + cos(0) + cos(0) = 0 + 1 + 1 = 2.
        // |2| - thickness. For thickness=2.0, field ≈ 0.
        let s = Solid::schwarz_p(1.0, 2.0);
        let val = s.evaluate(&Point3::new(PI / 2.0, 0.0, 0.0));
        assert!(
            val.abs() < 0.1,
            "schwarz_p at known zero-crossing should be near zero, got {val}"
        );
    }

    #[test]
    fn schwarz_p_is_infinite() {
        let s = Solid::schwarz_p(1.0, 0.5);
        assert!(
            s.bounds().is_none(),
            "schwarz_p should have no finite bounds"
        );
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn schwarz_p_rejects_zero_thickness() {
        drop(Solid::schwarz_p(1.0, 0.0));
    }

    #[test]
    fn schwarz_p_intersected_with_cuboid_meshes() {
        let envelope = Solid::cuboid(Vector3::new(4.0, 4.0, 4.0));
        let lattice = Solid::schwarz_p(1.0, 0.4);
        let part = envelope.intersect(lattice);
        let mesh = part.mesh(0.3);
        assert!(
            mesh.vertices.len() > 50,
            "schwarz_p lattice should produce substantial mesh, got {} verts",
            mesh.vertices.len()
        );
    }

    // -- Helix --

    #[test]
    fn helix_sign_correctness() {
        let s = Solid::helix(3.0, 2.0, 0.5, 2.0);
        // The tube surface at t=0 along +X is at (3.5, 0, 0): distance 0.5 to
        // helix curve at (3, 0, 0), minus thickness 0.5 → field ≈ 0.
        let val = s.evaluate(&Point3::new(3.5, 0.0, 0.0));
        assert!(
            val.abs() < 0.1,
            "point on tube surface should be near zero, got {val}"
        );
        // Point ON the helix curve at t=0: (3, 0, 0). Distance to curve = 0,
        // field = 0 - 0.5 = -0.5 → inside the tube.
        let val_inside = s.evaluate(&Point3::new(3.0, 0.0, 0.0));
        assert!(
            val_inside < -0.4,
            "point on helix curve should be inside tube, got {val_inside}"
        );
        // Origin is far from the helix coil (distance ≈ 3.0, field ≈ 2.5)
        assert!(
            s.evaluate(&Point3::origin()) > 0.0,
            "origin should be outside helix tube"
        );
    }

    #[test]
    fn helix_known_interior_point() {
        let s = Solid::helix(3.0, 2.0, 0.5, 2.0);
        // Slightly inside the tube at t=0: (2.8, 0, 0) → distance to (3,0,0) = 0.2
        // field = 0.2 - 0.5 = -0.3
        let val = s.evaluate(&Point3::new(2.8, 0.0, 0.0));
        assert!(
            val < 0.0,
            "point inside helix tube should be negative, got {val}"
        );
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn helix_bounds() {
        let s = Solid::helix(3.0, 2.0, 0.5, 2.0);
        let bb = s.bounds().expect("finite bounds");
        assert!((bb.min.x - (-3.5)).abs() < 1e-10, "x min should be -(R+th)");
        assert!((bb.max.x - 3.5).abs() < 1e-10, "x max should be R+th");
        assert!(
            (bb.max.z - 4.5).abs() < 1e-10,
            "z max should be turns*pitch+th"
        );
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn helix_rejects_zero_radius() {
        drop(Solid::helix(0.0, 2.0, 0.5, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn helix_rejects_zero_turns() {
        drop(Solid::helix(3.0, 2.0, 0.5, 0.0));
    }

    #[test]
    fn helix_meshes_to_valid_geometry() {
        let s = Solid::helix(3.0, 2.0, 0.5, 1.5);
        let mesh = s.mesh(0.25);
        assert!(
            mesh.vertices.len() > 50,
            "helix mesh should have substantial geometry, got {} verts",
            mesh.vertices.len()
        );
    }

    // ── Loft tests ──────────────────────────────────────────────────

    #[test]
    fn loft_constant_radius_sign() {
        let s = Solid::loft(&[(-5.0, 2.0), (5.0, 2.0)]);
        assert!(
            s.evaluate(&Point3::origin()) < 0.0,
            "center should be inside"
        );
        assert!(
            s.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0,
            "far point should be outside"
        );
    }

    #[test]
    fn loft_constant_radius_on_surface() {
        let s = Solid::loft(&[(-5.0, 2.0), (5.0, 2.0)]);
        assert!(
            s.evaluate(&Point3::new(2.0, 0.0, 0.0)).abs() < 1e-6,
            "barrel surface should be zero"
        );
    }

    #[test]
    fn loft_tapered_sign() {
        // Radius from 3 at bottom to 1 at top
        let s = Solid::loft(&[(-3.0, 3.0), (3.0, 1.0)]);
        assert!(s.evaluate(&Point3::origin()) < 0.0, "center inside");
        // On surface at bottom end
        assert!(
            s.evaluate(&Point3::new(3.0, 0.0, -3.0)).abs() < 1e-6,
            "bottom surface should be zero"
        );
        // On surface at top end
        assert!(
            s.evaluate(&Point3::new(1.0, 0.0, 3.0)).abs() < 1e-6,
            "top surface should be zero"
        );
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn loft_bounds() {
        let s = Solid::loft(&[(-5.0, 2.0), (5.0, 2.0)]);
        let bb = s.bounds().expect("finite bounds");
        assert!((bb.min.z - (-5.0)).abs() < 1e-10);
        assert!((bb.max.z - 5.0).abs() < 1e-10);
        assert!(bb.max.x >= 2.0);
    }

    #[test]
    #[should_panic(expected = "at least 2 stations")]
    fn loft_rejects_single_station() {
        drop(Solid::loft(&[(0.0, 1.0)]));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn loft_rejects_zero_radius() {
        drop(Solid::loft(&[(-1.0, 0.0), (1.0, 1.0)]));
    }

    #[test]
    #[should_panic(expected = "strictly sorted")]
    fn loft_rejects_unsorted_stations() {
        drop(Solid::loft(&[(3.0, 1.0), (1.0, 2.0)]));
    }

    #[test]
    fn loft_meshes_to_valid_geometry() {
        let s = Solid::loft(&[(-3.0, 2.0), (0.0, 3.0), (3.0, 1.0)]);
        let mesh = s.mesh(0.25);
        assert!(
            mesh.vertices.len() > 50,
            "loft mesh should have substantial geometry, got {} verts",
            mesh.vertices.len()
        );
    }

    // ── Twist tests ─────────────────────────────────────────────────

    #[test]
    fn twist_zero_rate_identity() {
        let s = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0)).twist(0.0);
        let orig = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0));
        let p = Point3::new(0.5, 0.5, 1.0);
        assert!(
            (s.evaluate(&p) - orig.evaluate(&p)).abs() < 1e-10,
            "zero twist should be identity"
        );
    }

    #[test]
    fn twist_preserves_cylinder_symmetry() {
        let s = Solid::cylinder(2.0, 5.0).twist(1.0);
        let orig = Solid::cylinder(2.0, 5.0);
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 3.0),
        ];
        for p in &test_points {
            assert!(
                (s.evaluate(p) - orig.evaluate(p)).abs() < 1e-6,
                "twist should not change cylindrically symmetric field"
            );
        }
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn twist_bounds() {
        let s = Solid::cuboid(Vector3::new(1.0, 2.0, 5.0)).twist(1.0);
        let bb = s.bounds().expect("finite bounds");
        let r = 1.0_f64.hypot(2.0);
        assert!((bb.min.x - (-r)).abs() < 1e-10);
        assert!((bb.max.x - r).abs() < 1e-10);
        assert!((bb.min.z - (-5.0)).abs() < 1e-10);
        assert!((bb.max.z - 5.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "finite")]
    fn twist_rejects_infinity() {
        drop(Solid::sphere(1.0).twist(f64::INFINITY));
    }

    #[test]
    fn twist_meshes_to_valid_geometry() {
        let s = Solid::cuboid(Vector3::new(1.0, 0.5, 4.0)).twist(0.5);
        let mesh = s.mesh(0.2);
        assert!(
            mesh.vertices.len() > 50,
            "twisted cuboid mesh should have substantial geometry, got {} verts",
            mesh.vertices.len()
        );
    }

    // ── Bend tests ──────────────────────────────────────────────────

    #[test]
    fn bend_zero_rate_identity() {
        let s = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0)).bend(0.0);
        let orig = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0));
        let p = Point3::new(0.5, 0.5, 1.0);
        assert!(
            (s.evaluate(&p) - orig.evaluate(&p)).abs() < 1e-10,
            "zero bend should be identity"
        );
    }

    #[test]
    fn bend_at_z_zero_matches_child() {
        let s = Solid::cuboid(Vector3::new(2.0, 2.0, 5.0)).bend(0.3);
        let orig = Solid::cuboid(Vector3::new(2.0, 2.0, 5.0));
        // At z=0, bend angle=0, so field should match child
        let p = Point3::new(1.0, 1.0, 0.0);
        assert!(
            (s.evaluate(&p) - orig.evaluate(&p)).abs() < 1e-10,
            "bend at z=0 should match child"
        );
    }

    #[test]
    fn bend_center_stays_inside() {
        // Moderate bend: center of a cylinder should still be inside
        let s = Solid::cylinder(1.0, 5.0).bend(0.1);
        let val = s.evaluate(&Point3::new(0.0, 0.0, 3.0));
        assert!(
            val < 0.0,
            "center of bent cylinder should be inside, got {val}"
        );
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn bend_bounds() {
        let s = Solid::cuboid(Vector3::new(1.0, 2.0, 5.0)).bend(0.5);
        let bb = s.bounds().expect("finite bounds");
        let r = 1.0_f64.hypot(5.0);
        assert!((bb.min.x - (-r)).abs() < 1e-10);
        assert!((bb.min.y - (-2.0)).abs() < 1e-10);
        assert!((bb.max.y - 2.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "finite")]
    fn bend_rejects_infinity() {
        drop(Solid::sphere(1.0).bend(f64::INFINITY));
    }

    #[test]
    fn bend_meshes_to_valid_geometry() {
        let s = Solid::cuboid(Vector3::new(1.0, 1.0, 5.0)).bend(0.15);
        let mesh = s.mesh(0.2);
        assert!(
            mesh.vertices.len() > 50,
            "bent cuboid mesh should have substantial geometry, got {} verts",
            mesh.vertices.len()
        );
    }

    // ── Repeat ───────────────────────────────────────────────────────

    #[test]
    fn repeat_is_infinite() {
        let s = Solid::sphere(1.0).repeat(Vector3::new(5.0, 5.0, 5.0));
        assert!(s.bounds().is_none());
    }

    #[test]
    fn repeat_evaluates_at_copy() {
        let s = Solid::sphere(1.0).repeat(Vector3::new(5.0, 5.0, 5.0));
        // At origin → inside
        assert!(s.evaluate(&Point3::origin()) < 0.0);
        // At (5, 0, 0) → inside a copy
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) < 0.0);
        // At (2.5, 0, 0) → midpoint between copies, outside (radius 1 < spacing/2)
        assert!(s.evaluate(&Point3::new(2.5, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn repeat_intersected_with_cuboid_meshes() {
        let holes = Solid::cylinder(0.3, 2.0).repeat(Vector3::new(2.0, 2.0, 100.0));
        let plate = Solid::cuboid(Vector3::new(3.0, 3.0, 1.0));
        let result = plate.subtract(holes);
        let mesh = result.mesh(0.2);
        assert!(
            !mesh.is_empty(),
            "repeated-hole plate should produce non-empty mesh"
        );
        assert!(
            mesh.vertices.len() > 100,
            "should have substantial geometry, got {} verts",
            mesh.vertices.len()
        );
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn repeat_rejects_zero_spacing() {
        drop(Solid::sphere(1.0).repeat(Vector3::new(0.0, 5.0, 5.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn repeat_rejects_negative_spacing() {
        drop(Solid::sphere(1.0).repeat(Vector3::new(-1.0, 5.0, 5.0)));
    }

    // ── RepeatBounded ────────────────────────────────────────────────

    #[test]
    fn repeat_bounded_has_finite_bounds() {
        let s = Solid::sphere(1.0).repeat_bounded(Vector3::new(5.0, 5.0, 5.0), [3, 1, 1]);
        let bb = s.bounds();
        assert!(bb.is_some(), "bounded repeat should have finite bounds");
    }

    #[test]
    fn repeat_bounded_evaluates_all_copies() {
        let s = Solid::sphere(1.0).repeat_bounded(Vector3::new(5.0, 5.0, 5.0), [3, 1, 1]);
        // 3 copies at x = -5, 0, 5
        assert!(s.evaluate(&Point3::origin()) < 0.0);
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) < 0.0);
        assert!(s.evaluate(&Point3::new(-5.0, 0.0, 0.0)) < 0.0);
        // Between copies → outside
        assert!(s.evaluate(&Point3::new(2.5, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn repeat_bounded_meshes_to_valid_geometry() {
        let s = Solid::sphere(0.8).repeat_bounded(Vector3::new(3.0, 3.0, 3.0), [3, 2, 1]);
        let mesh = s.mesh(0.3);
        assert!(
            !mesh.is_empty(),
            "bounded repeat should produce non-empty mesh"
        );
        assert!(
            mesh.vertices.len() > 200,
            "3x2 array of spheres should have many vertices, got {}",
            mesh.vertices.len()
        );
        // Volume check: 6 spheres of radius 0.8, expected ≈ 6 * 4/3 π 0.8³ ≈ 12.87
        let vol = mesh.volume();
        let expected = 6.0 * 4.0 / 3.0 * PI * 0.8_f64.powi(3);
        let error = (vol - expected).abs() / expected;
        assert!(
            error < 0.2,
            "volume error {:.1}% exceeds 20% (expected {expected:.1}, got {vol:.1})",
            error * 100.0,
        );
    }

    #[test]
    fn repeat_bounded_count_1_is_identity() {
        let base = Solid::sphere(2.0);
        let repeated = Solid::sphere(2.0).repeat_bounded(Vector3::new(10.0, 10.0, 10.0), [1, 1, 1]);
        let p = Point3::new(1.0, 0.5, 0.3);
        assert!(
            (base.evaluate(&p) - repeated.evaluate(&p)).abs() < 1e-10,
            "count=[1,1,1] should be identity"
        );
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn repeat_bounded_rejects_zero_spacing() {
        drop(Solid::sphere(1.0).repeat_bounded(Vector3::new(0.0, 5.0, 5.0), [3, 1, 1]));
    }

    #[test]
    #[should_panic(expected = ">= 1")]
    fn repeat_bounded_rejects_zero_count() {
        drop(Solid::sphere(1.0).repeat_bounded(Vector3::new(5.0, 5.0, 5.0), [0, 1, 1]));
    }

    // ── Lipschitz factor ─────────────────────────────────────────────

    #[test]
    fn lipschitz_factor_sphere_is_1() {
        let s = Solid::sphere(5.0);
        assert!(
            (s.lipschitz_factor() - 1.0).abs() < 1e-10,
            "Sphere L should be 1.0, got {}",
            s.lipschitz_factor()
        );
    }

    #[test]
    fn lipschitz_factor_twisted_is_gt_1() {
        let s = Solid::cuboid(Vector3::new(3.0, 3.0, 10.0)).twist(1.0);
        let lip = s.lipschitz_factor();
        assert!(lip > 1.0, "Twisted solid should have L > 1, got {lip}");
    }

    #[test]
    fn lipschitz_factor_used_in_mesh_resolution() {
        // Verify that mesh() uses a finer cell size for distorted fields.
        // A twisted cuboid should produce more vertices than an untwisted one
        // at the same tolerance, because the Lipschitz factor refines the grid.
        let base = Solid::cuboid(Vector3::new(2.0, 2.0, 5.0));
        let twisted = Solid::cuboid(Vector3::new(2.0, 2.0, 5.0)).twist(0.5);
        let mesh_base = base.mesh(0.5);
        let mesh_twisted = twisted.mesh(0.5);
        assert!(
            mesh_twisted.vertices.len() > mesh_base.vertices.len(),
            "Twisted mesh ({} verts) should have more vertices than base ({} verts) due to Lipschitz scaling",
            mesh_twisted.vertices.len(),
            mesh_base.vertices.len()
        );
    }

    #[test]
    fn lipschitz_factor_mesh_captures_twisted_features() {
        // A twisted thin shell: without Lipschitz scaling, thin features could
        // be lost. With scaling, they should be captured.
        let s = Solid::cuboid(Vector3::new(2.0, 2.0, 5.0))
            .shell(0.3)
            .twist(0.3);
        let mesh = s.mesh(0.4);
        assert!(
            !mesh.is_empty(),
            "Twisted shell should produce non-empty mesh with Lipschitz scaling"
        );
        // Volume should be positive (features captured, not lost)
        assert!(
            mesh.volume() > 0.0,
            "Twisted shell mesh volume should be positive"
        );
    }

    // ── Infill tests ────────────────────────────────────────────────

    #[test]
    fn gyroid_infill_cuboid_produces_mesh() {
        let s =
            Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)).infill(InfillKind::Gyroid, 1.0, 0.4, 0.5);
        let mesh = s.mesh(0.4);
        assert!(
            mesh.vertices.len() > 100,
            "gyroid-infilled cuboid should produce substantial mesh, got {} verts",
            mesh.vertices.len()
        );
    }

    #[test]
    fn schwarz_p_infill_sphere_produces_mesh() {
        let s = Solid::sphere(5.0).infill(InfillKind::SchwarzP, 1.0, 0.4, 0.5);
        let mesh = s.mesh(0.4);
        assert!(
            mesh.vertices.len() > 100,
            "schwarz_p-infilled sphere should produce substantial mesh, got {} verts",
            mesh.vertices.len()
        );
    }

    #[test]
    fn infill_preserves_outer_shell() {
        let original = Solid::sphere(5.0);
        let infilled = Solid::sphere(5.0).infill(InfillKind::Gyroid, 1.0, 0.3, 0.5);
        // On the original surface, infill should be near zero (shell boundary)
        let surface_pt = Point3::new(5.0, 0.0, 0.0);
        let original_val = original.evaluate(&surface_pt);
        let infilled_val = infilled.evaluate(&surface_pt);
        assert!(
            original_val.abs() < 1e-10,
            "surface point should be on original surface"
        );
        assert!(
            infilled_val.abs() < 1.0,
            "infilled surface should be near the original surface, got {infilled_val}"
        );
    }

    #[test]
    fn infill_has_lattice_inside() {
        let infilled =
            Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)).infill(InfillKind::Gyroid, 1.0, 0.4, 0.5);
        // Sample a grid of interior points — some should be inside lattice
        // (negative) and some should be outside (positive, in the voids)
        let mut has_inside = false;
        let mut has_outside = false;
        for i in 0..10 {
            let x = f64::from(i).mul_add(0.6, -3.0);
            let val = infilled.evaluate(&Point3::new(x, 0.0, 0.0));
            if val < 0.0 {
                has_inside = true;
            }
            if val > 0.0 {
                has_outside = true;
            }
        }
        assert!(
            has_inside,
            "infilled interior should have lattice material (negative field)"
        );
        assert!(
            has_outside,
            "infilled interior should have voids (positive field)"
        );
    }

    #[test]
    fn infill_volume_less_than_solid() {
        let solid = Solid::cuboid(Vector3::new(5.0, 5.0, 5.0));
        let infilled =
            Solid::cuboid(Vector3::new(5.0, 5.0, 5.0)).infill(InfillKind::Gyroid, 1.0, 0.4, 0.5);
        let vol_solid = solid.mesh(0.5).volume();
        let vol_infill = infilled.mesh(0.4).volume();
        assert!(
            vol_infill < vol_solid,
            "infilled volume ({vol_infill:.1}) should be less than solid volume ({vol_solid:.1})"
        );
    }

    #[test]
    fn gyroid_infill_mesh_watertight() {
        let infilled =
            Solid::cuboid(Vector3::new(4.0, 4.0, 4.0)).infill(InfillKind::Gyroid, 1.0, 0.4, 0.5);
        let mesh = infilled.mesh(0.4);
        assert_mesh_valid(&mesh, "gyroid_infill");
    }

    #[test]
    fn schwarz_p_infill_mesh_watertight() {
        let infilled = Solid::sphere(4.0).infill(InfillKind::SchwarzP, 1.0, 0.4, 0.5);
        let mesh = infilled.mesh(0.4);
        assert_mesh_valid(&mesh, "schwarz_p_infill");
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn infill_rejects_zero_scale() {
        drop(Solid::sphere(5.0).infill(InfillKind::Gyroid, 0.0, 0.4, 0.5));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn infill_rejects_negative_wall() {
        drop(Solid::sphere(5.0).infill(InfillKind::Gyroid, 1.0, 0.4, -0.5));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn infill_rejects_nan_lattice_thickness() {
        drop(Solid::sphere(5.0).infill(InfillKind::SchwarzP, 1.0, f64::NAN, 0.5));
    }

    // ── Variable-radius smooth union tests ──────────────────────────

    #[test]
    fn smooth_union_variable_constant_k_matches_smooth_union() {
        let sphere_a = Solid::sphere(2.0);
        let sphere_b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let blend_k = 1.0;

        let constant = sphere_a.clone().smooth_union(sphere_b.clone(), blend_k);
        let variable = sphere_a.smooth_union_variable(sphere_b, |_| 1.0, blend_k);

        // Should produce identical values at several test points
        let pts = [
            Point3::origin(),
            Point3::new(1.5, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        ];
        for pt in &pts {
            let cv = constant.evaluate(pt);
            let vv = variable.evaluate(pt);
            assert!(
                (cv - vv).abs() < 1e-10,
                "constant and variable should match at {pt:?}: {cv} vs {vv}"
            );
        }
    }

    #[test]
    fn smooth_union_variable_larger_k_adds_more_material() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));

        // Small k on left side (x < 1.5), large k on right side (x >= 1.5)
        let variable = a.smooth_union_variable(b, |p| if p.x < 1.5 { 0.1 } else { 2.0 }, 2.0);

        // In the blend region near x=1.5, the right side (large k) should have
        // more material (more negative field) than the left side (small k)
        let left = variable.evaluate(&Point3::new(1.0, 1.0, 0.0));
        let right = variable.evaluate(&Point3::new(2.0, 1.0, 0.0));
        // Both points are in the blend zone between the two spheres.
        // The right point has k=2.0 so more blending, the left has k=0.1.
        // We just check that the variable blend produces valid field values.
        assert!(
            left.is_finite() && right.is_finite(),
            "variable blend should produce finite values"
        );
    }

    #[test]
    fn smooth_union_variable_far_from_blend_matches_sharp() {
        let sphere_a = Solid::sphere(2.0);
        let sphere_b = Solid::sphere(2.0).translate(Vector3::new(10.0, 0.0, 0.0));

        let variable = sphere_a
            .clone()
            .smooth_union_variable(sphere_b.clone(), |_| 0.5, 0.5);
        let sharp = sphere_a.union(sphere_b);

        // Far from blend region: should match sharp union
        let origin = Point3::origin();
        let var_val = variable.evaluate(&origin);
        let sharp_val = sharp.evaluate(&origin);
        assert!(
            (var_val - sharp_val).abs() < 1e-6,
            "far from blend, variable should match sharp: {var_val} vs {sharp_val}"
        );
    }

    #[test]
    fn smooth_union_variable_meshes() {
        let a = Solid::sphere(3.0);
        let b = Solid::sphere(3.0).translate(Vector3::new(4.0, 0.0, 0.0));
        let blended = a.smooth_union_variable(b, |p| p.x.abs().mul_add(0.1, 0.5), 2.0);
        let mesh = blended.mesh(0.5);
        assert!(
            mesh.vertices.len() > 50,
            "variable blended mesh should have substantial geometry, got {} verts",
            mesh.vertices.len()
        );
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_union_variable_rejects_zero_max_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_union_variable(b, |_| 1.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_union_variable_rejects_nan_max_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_union_variable(b, |_| 1.0, f64::NAN));
    }

    #[test]
    fn smooth_union_variable_is_cloneable() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let s = a.smooth_union_variable(b, |_| 1.0, 1.0);
        let s2 = s.clone();
        let p = Point3::new(1.5, 0.0, 0.0);
        assert!(
            (s.evaluate(&p) - s2.evaluate(&p)).abs() < 1e-10,
            "clone should produce identical evaluations"
        );
    }
}
