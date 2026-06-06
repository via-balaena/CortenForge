//! Parameterized constructors and parameter access/gradients for [`Solid`].

use nalgebra::Point3;

use super::Solid;
use crate::field_node::{FieldNode, Val};
use crate::param::ParamRef;

impl Solid {
    // ── Parameterized constructors ────────────────────────────────────

    /// Sphere with parameterized radius.
    ///
    /// Like [`sphere`](Self::sphere), but the radius is a design variable
    /// that can be updated via [`ParamStore::set`](crate::ParamStore::set)
    /// without rebuilding the tree.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite.
    #[must_use]
    pub fn sphere_p(radius: ParamRef) -> Self {
        let r = radius.value();
        assert!(
            r > 0.0 && r.is_finite(),
            "sphere_p radius must be positive and finite, got {r}"
        );
        Self {
            node: FieldNode::Sphere {
                radius: Val::from(radius),
            },
        }
    }

    /// Smooth union with parameterized blend radius.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite.
    #[must_use]
    pub fn smooth_union_p(self, other: Self, k: ParamRef) -> Self {
        let kv = k.value();
        assert!(
            kv > 0.0 && kv.is_finite(),
            "smooth_union_p blend radius k must be positive and finite, got {kv}"
        );
        Self {
            node: FieldNode::SmoothUnion(Box::new(self.node), Box::new(other.node), Val::from(k)),
        }
    }

    /// Smooth subtraction with parameterized blend radius.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite.
    #[must_use]
    pub fn smooth_subtract_p(self, other: Self, k: ParamRef) -> Self {
        let kv = k.value();
        assert!(
            kv > 0.0 && kv.is_finite(),
            "smooth_subtract_p blend radius k must be positive and finite, got {kv}"
        );
        Self {
            node: FieldNode::SmoothSubtract(
                Box::new(self.node),
                Box::new(other.node),
                Val::from(k),
            ),
        }
    }

    /// Smooth intersection with parameterized blend radius.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite.
    #[must_use]
    pub fn smooth_intersect_p(self, other: Self, k: ParamRef) -> Self {
        let kv = k.value();
        assert!(
            kv > 0.0 && kv.is_finite(),
            "smooth_intersect_p blend radius k must be positive and finite, got {kv}"
        );
        Self {
            node: FieldNode::SmoothIntersect(
                Box::new(self.node),
                Box::new(other.node),
                Val::from(k),
            ),
        }
    }

    /// Shell with parameterized wall thickness.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite.
    #[must_use]
    pub fn shell_p(self, thickness: ParamRef) -> Self {
        let v = thickness.value();
        assert!(
            v > 0.0 && v.is_finite(),
            "shell_p thickness must be positive and finite, got {v}"
        );
        Self {
            node: FieldNode::Shell(Box::new(self.node), Val::from(thickness)),
        }
    }

    /// Round with parameterized rounding radius.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not positive and finite.
    #[must_use]
    pub fn round_p(self, radius: ParamRef) -> Self {
        let v = radius.value();
        assert!(
            v > 0.0 && v.is_finite(),
            "round_p radius must be positive and finite, got {v}"
        );
        Self {
            node: FieldNode::Round(Box::new(self.node), Val::from(radius)),
        }
    }

    /// Offset with parameterized distance.
    ///
    /// # Panics
    ///
    /// Panics if the current parameter value is not finite.
    #[must_use]
    pub fn offset_p(self, distance: ParamRef) -> Self {
        let v = distance.value();
        assert!(v.is_finite(), "offset_p distance must be finite, got {v}");
        Self {
            node: FieldNode::Offset(Box::new(self.node), Val::from(distance)),
        }
    }

    // ── Parameter access ──────────────────────────────────────────────

    /// Set a named design parameter. Searches the expression tree for the
    /// parameter's store and updates the value.
    ///
    /// Takes `&self` (not `&mut self`) — the parameter store uses interior
    /// mutability, so concurrent evaluation threads see the update immediately.
    ///
    /// # Panics
    ///
    /// Panics if the parameter name is not found in this solid's tree.
    pub fn set_param(&self, name: &str, value: f64) {
        let result = self.node.find_param(name);
        assert!(
            result.is_some(),
            "parameter '{name}' not found in this solid"
        );
        if let Some((id, store)) = result {
            store.set_by_id(id, value);
        }
    }

    /// Get a named design parameter's current value.
    ///
    /// Returns `None` if the parameter is not found in this solid's tree.
    #[must_use]
    pub fn get_param(&self, name: &str) -> Option<f64> {
        self.node
            .find_param(name)
            .map(|(id, store)| store.get_by_id(id))
    }

    /// List all design parameter names reachable from this solid's tree.
    #[must_use]
    pub fn param_names(&self) -> Vec<String> {
        self.node
            .collect_params()
            .into_iter()
            .map(|(name, _, _)| name)
            .collect()
    }

    // ── Parameter gradients ──────────────────────────────────────────

    /// Compute `∂f/∂θ` at a point for a named design parameter.
    ///
    /// Returns the derivative of the scalar field with respect to the named
    /// parameter, computed analytically via chain rule through the expression
    /// tree.
    ///
    /// # Panics
    ///
    /// Panics if the parameter name is not found in this solid's tree.
    #[must_use]
    pub fn param_gradient(&self, p: &Point3<f64>, param_name: &str) -> f64 {
        let result = self.node.find_param(param_name);
        assert!(
            result.is_some(),
            "parameter '{param_name}' not found in this solid"
        );
        if let Some((id, _)) = result {
            self.node.param_gradient(p, id)
        } else {
            0.0
        }
    }

    /// Compute `∂f/∂θ_i` for all design parameters at a point.
    ///
    /// Returns a vector of `(name, gradient)` pairs — one per parameter
    /// reachable from this solid's expression tree.
    #[must_use]
    pub fn param_gradients(&self, p: &Point3<f64>) -> Vec<(String, f64)> {
        self.node
            .collect_params()
            .into_iter()
            .map(|(name, id, _)| {
                let grad = self.node.param_gradient(p, id);
                (name, grad)
            })
            .collect()
    }
}
