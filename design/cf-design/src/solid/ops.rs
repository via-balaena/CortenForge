//! Boolean, transform, and domain operations on [`Solid`].

use std::sync::Arc;

use cf_geometry::Aabb;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::{InfillKind, Solid};
use crate::Sdf;
use crate::field_node::{FieldNode, UserEvalFn, UserIntervalFn, Val};
use crate::param::ParamRef;

impl Solid {
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
            node: FieldNode::SmoothUnion(Box::new(self.node), Box::new(other.node), Val::from(k)),
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
            node: FieldNode::SmoothSubtract(
                Box::new(self.node),
                Box::new(other.node),
                Val::from(k),
            ),
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
            node: FieldNode::SmoothIntersect(
                Box::new(self.node),
                Box::new(other.node),
                Val::from(k),
            ),
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
            node: FieldNode::SmoothUnionAll(nodes, Val::from(k)),
        }
    }

    /// N-ary smooth union with parameterized blend radius.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite, or
    /// if `solids` is empty.
    #[must_use]
    pub fn smooth_union_all_p(solids: Vec<Self>, k: ParamRef) -> Self {
        let kv = k.value();
        assert!(
            !solids.is_empty(),
            "smooth_union_all_p requires at least one solid"
        );
        assert!(
            kv > 0.0 && kv.is_finite(),
            "smooth_union_all_p blend radius k must be positive and finite, got {kv}"
        );
        let nodes: Vec<FieldNode> = solids.into_iter().map(|s| s.node).collect();
        Self {
            node: FieldNode::SmoothUnionAll(nodes, Val::from(k)),
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
            node: FieldNode::Shell(Box::new(self.node), Val::from(thickness)),
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
            node: FieldNode::Round(Box::new(self.node), Val::from(radius)),
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
            node: FieldNode::Offset(Box::new(self.node), Val::from(distance)),
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

    /// Promote any [`Sdf`] implementor into a typed [`Solid`] leaf.
    ///
    /// Sugar over [`Self::user_fn`] for the common pattern of bridging an
    /// externally-defined signed-distance function — a scan-derived
    /// `mesh_sdf::Signed`, a custom analytic function
    /// wrapped behind an `impl Sdf` newtype, or any other crate's SDF
    /// type that satisfies the [`Sdf`] contract — into the typed-`Solid`
    /// expression tree, so it composes with parametric primitives via
    /// `union` / `subtract` / `intersect` like any other `Solid`.
    ///
    /// `bounds` is the conservative axis-aligned bounding box of the
    /// SDF's zero set; the mesher uses it to define the evaluation
    /// domain. Loose bounds are safe (mesher walks more lattice cells,
    /// no correctness impact); tight bounds improve meshing performance.
    ///
    /// Octree pruning is disabled for this subtree (interval evaluation
    /// returns `(-∞, +∞)`), inherited from [`Self::user_fn`]. Consumers
    /// that need pruning should use [`Self::user_fn_with_interval`]
    /// directly with a hand-derived interval bound.
    ///
    /// The [`Sdf`] trait's `Send + Sync` supertrait + the `'static`
    /// bound here satisfy the closure-capture requirements of the
    /// underlying `FieldNode::UserFn` storage; every reasonable `Sdf`
    /// implementor (a pure function, a `Clone + Send + Sync` SDF
    /// struct, a composition of the same) meets these bounds
    /// naturally.
    #[must_use]
    pub fn from_sdf<S: Sdf + 'static>(sdf: S, bounds: Aabb) -> Self {
        Self::user_fn(move |p| sdf.eval(p), bounds)
    }
}
