//! Material and manufacturing process types.
//!
//! A [`Material`] describes the physical properties of a [`super::Part`]:
//! density, stiffness, color, and manufacturing process. These feed into
//! mass property computation (Session 11) and flex zone stiffness derivation
//! (Session 17).

/// Manufacturing process for a part.
///
/// Determines manufacturing constraints (layer height, minimum features)
/// and informs the validation system. Use the validated constructors
/// ([`fdm`](Self::fdm), [`sla`](Self::sla)) for processes with parameters.
#[derive(Debug, Clone, PartialEq)]
pub enum ManufacturingProcess {
    /// Fused deposition modeling (FDM / FFF).
    Fdm {
        /// Layer height in mm. Must be positive.
        layer_height: f64,
        /// Infill ratio in (0.0, 1.0]. 1.0 = solid.
        infill: f64,
    },
    /// Stereolithography (SLA / resin).
    Sla {
        /// Layer height in mm. Must be positive.
        layer_height: f64,
    },
    /// Selective laser sintering (SLS / powder bed).
    Sls,
    /// CNC machining or similar subtractive process.
    Machined,
}

impl ManufacturingProcess {
    /// Create an FDM process with validated parameters.
    ///
    /// # Panics
    ///
    /// Panics if `layer_height` is not positive/finite or `infill` is not
    /// in (0.0, 1.0].
    #[must_use]
    pub fn fdm(layer_height: f64, infill: f64) -> Self {
        assert!(
            layer_height > 0.0 && layer_height.is_finite(),
            "FDM layer_height must be positive and finite, got {layer_height}"
        );
        assert!(
            infill > 0.0 && infill <= 1.0 && infill.is_finite(),
            "FDM infill must be in (0.0, 1.0], got {infill}"
        );
        Self::Fdm {
            layer_height,
            infill,
        }
    }

    /// Create an SLA process with validated parameters.
    ///
    /// # Panics
    ///
    /// Panics if `layer_height` is not positive and finite.
    #[must_use]
    pub fn sla(layer_height: f64) -> Self {
        assert!(
            layer_height > 0.0 && layer_height.is_finite(),
            "SLA layer_height must be positive and finite, got {layer_height}"
        );
        Self::Sla { layer_height }
    }
}

/// Physical material properties for a part.
///
/// Required: `name` and `density`. Optional fields are set via builder methods
/// ([`with_youngs_modulus`](Self::with_youngs_modulus),
/// [`with_color`](Self::with_color),
/// [`with_process`](Self::with_process)).
///
/// # Example
///
/// ```
/// use cf_design::Material;
/// use cf_design::ManufacturingProcess;
///
/// let pla = Material::new("PLA", 1250.0)
///     .with_youngs_modulus(3.5e9)
///     .with_color([0.8, 0.2, 0.1, 1.0])
///     .with_process(ManufacturingProcess::fdm(0.2, 0.2));
/// ```
#[derive(Debug, Clone)]
pub struct Material {
    /// Human-readable material name (e.g., "PLA", "TPU 95A").
    pub name: String,
    /// Density in kg/m³. Must be positive and finite.
    pub density: f64,
    /// Young's modulus in Pa. Used for flex zone spring-damper stiffness.
    pub youngs_modulus: Option<f64>,
    /// RGBA color for visualization. Each component typically in [0, 1].
    pub color: Option<[f64; 4]>,
    /// Manufacturing process. Determines manufacturing constraints.
    pub process: Option<ManufacturingProcess>,
}

impl Material {
    /// Create a material with the given name and density.
    ///
    /// # Panics
    ///
    /// Panics if `density` is not positive and finite.
    #[must_use]
    pub fn new(name: impl Into<String>, density: f64) -> Self {
        assert!(
            density > 0.0 && density.is_finite(),
            "density must be positive and finite, got {density}"
        );
        Self {
            name: name.into(),
            density,
            youngs_modulus: None,
            color: None,
            process: None,
        }
    }

    /// Set Young's modulus (Pa).
    ///
    /// # Panics
    ///
    /// Panics if the value is not positive and finite.
    #[must_use]
    pub fn with_youngs_modulus(mut self, e: f64) -> Self {
        assert!(
            e > 0.0 && e.is_finite(),
            "Young's modulus must be positive and finite, got {e}"
        );
        self.youngs_modulus = Some(e);
        self
    }

    /// Set RGBA color for visualization.
    #[must_use]
    pub const fn with_color(mut self, rgba: [f64; 4]) -> Self {
        self.color = Some(rgba);
        self
    }

    /// Set manufacturing process.
    #[must_use]
    pub const fn with_process(mut self, process: ManufacturingProcess) -> Self {
        self.process = Some(process);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn material_new_valid() {
        let m = Material::new("PLA", 1250.0);
        assert_eq!(m.name, "PLA");
        assert!((m.density - 1250.0).abs() < f64::EPSILON);
        assert!(m.youngs_modulus.is_none());
        assert!(m.color.is_none());
        assert!(m.process.is_none());
    }

    #[test]
    fn material_with_all_optional_fields() {
        let m = Material::new("PLA", 1250.0)
            .with_youngs_modulus(3.5e9)
            .with_color([0.8, 0.2, 0.1, 1.0])
            .with_process(ManufacturingProcess::fdm(0.2, 0.2));

        assert_eq!(m.youngs_modulus, Some(3.5e9));
        assert_eq!(m.color, Some([0.8, 0.2, 0.1, 1.0]));
        assert!(matches!(m.process, Some(ManufacturingProcess::Fdm { .. })));
    }

    #[test]
    #[should_panic(expected = "density must be positive")]
    fn material_rejects_zero_density() {
        let _m = Material::new("bad", 0.0);
    }

    #[test]
    #[should_panic(expected = "density must be positive")]
    fn material_rejects_negative_density() {
        let _m = Material::new("bad", -100.0);
    }

    #[test]
    #[should_panic(expected = "density must be positive")]
    fn material_rejects_nan_density() {
        let _m = Material::new("bad", f64::NAN);
    }

    #[test]
    #[should_panic(expected = "density must be positive")]
    fn material_rejects_inf_density() {
        let _m = Material::new("bad", f64::INFINITY);
    }

    #[test]
    #[should_panic(expected = "Young's modulus must be positive")]
    fn material_rejects_negative_youngs_modulus() {
        let _m = Material::new("PLA", 1250.0).with_youngs_modulus(-1.0);
    }

    #[test]
    fn fdm_process_valid() {
        let p = ManufacturingProcess::fdm(0.2, 0.2);
        assert!(matches!(
            p,
            ManufacturingProcess::Fdm {
                layer_height,
                infill,
            } if (layer_height - 0.2).abs() < f64::EPSILON
              && (infill - 0.2).abs() < f64::EPSILON
        ));
    }

    #[test]
    fn fdm_solid_infill() {
        let p = ManufacturingProcess::fdm(0.1, 1.0);
        assert!(
            matches!(p, ManufacturingProcess::Fdm { infill, .. } if (infill - 1.0).abs() < f64::EPSILON)
        );
    }

    #[test]
    #[should_panic(expected = "FDM layer_height must be positive")]
    fn fdm_rejects_zero_layer_height() {
        let _p = ManufacturingProcess::fdm(0.0, 0.2);
    }

    #[test]
    #[should_panic(expected = "FDM infill must be in")]
    fn fdm_rejects_zero_infill() {
        let _p = ManufacturingProcess::fdm(0.2, 0.0);
    }

    #[test]
    #[should_panic(expected = "FDM infill must be in")]
    fn fdm_rejects_infill_above_one() {
        let _p = ManufacturingProcess::fdm(0.2, 1.1);
    }

    #[test]
    fn sla_process_valid() {
        let p = ManufacturingProcess::sla(0.05);
        assert!(matches!(
            p,
            ManufacturingProcess::Sla { layer_height } if (layer_height - 0.05).abs() < f64::EPSILON
        ));
    }

    #[test]
    #[should_panic(expected = "SLA layer_height must be positive")]
    fn sla_rejects_zero_layer_height() {
        let _p = ManufacturingProcess::sla(0.0);
    }

    #[test]
    fn sls_and_machined_constructible() {
        assert!(matches!(
            ManufacturingProcess::Sls,
            ManufacturingProcess::Sls
        ));
        assert!(matches!(
            ManufacturingProcess::Machined,
            ManufacturingProcess::Machined
        ));
    }
}
