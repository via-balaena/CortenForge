//! Material properties for deformable bodies.
//!
//! This module defines material properties that control the physical behavior
//! of deformable bodies, including stiffness, damping, and density.
//!
//! # Material Model
//!
//! The material model uses standard continuum mechanics parameters:
//!
//! - **Young's modulus (E)**: Measures stiffness (resistance to stretching)
//! - **Poisson's ratio (ν)**: Measures volume preservation (0-0.5)
//! - **Density (ρ)**: Mass per unit volume
//!
//! From these, we derive XPBD compliance parameters:
//!
//! - **Stretch compliance**: α = 1 / (E * A) for 1D, 1 / (E * h) for 2D
//! - **Bend compliance**: α = 1 / (E * I) where I is second moment of area
//! - **Volume compliance**: α = 1 / (K * V) where K is bulk modulus

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::error::{DeformableError, Result};

/// Material preset for common materials.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MaterialPreset {
    /// Soft rubber (like silicone).
    Rubber,
    /// Fabric/cloth.
    Cloth,
    /// Soft tissue (biological).
    SoftTissue,
    /// Muscle tissue.
    Muscle,
    /// Cartilage.
    Cartilage,
    /// Gelatin/jelly.
    Gelatin,
    /// Foam/sponge.
    Foam,
    /// Leather.
    Leather,
    /// Rope (natural fiber).
    Rope,
    /// Steel cable.
    SteelCable,
    /// Tendon (biological).
    Tendon,
    /// Paper.
    Paper,
    /// Plastic (flexible).
    FlexiblePlastic,
    /// Wood (soft).
    SoftWood,
}

/// Material properties for deformable bodies.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Material {
    /// Young's modulus in Pascals (Pa).
    /// Measures resistance to stretching.
    /// Higher values = stiffer material.
    pub youngs_modulus: f64,

    /// Poisson's ratio (dimensionless, 0 to 0.5).
    /// Measures how much material contracts perpendicular to stretch.
    /// 0 = no lateral contraction
    /// 0.5 = incompressible (volume preserving)
    pub poissons_ratio: f64,

    /// Density in kg/m³.
    pub density: f64,

    /// Damping coefficient for velocity-based damping.
    /// Higher values = more energy dissipation.
    pub damping: f64,

    /// Stretch stiffness multiplier (1.0 = use Young's modulus).
    /// Can be used to tune stretch resistance independently.
    pub stretch_factor: f64,

    /// Bending stiffness multiplier (1.0 = derived from Young's modulus).
    /// Can be used to tune bending resistance independently.
    pub bend_factor: f64,

    /// Volume preservation factor (1.0 = derived from bulk modulus).
    /// Can be used to tune volume preservation independently.
    pub volume_factor: f64,

    /// Friction coefficient for self-collision and contact.
    pub friction: f64,

    /// Whether this material is compressible.
    /// If true, volume constraints use finite compliance.
    /// If false, volume is strictly preserved.
    pub compressible: bool,
}

impl Default for Material {
    fn default() -> Self {
        Self::preset(MaterialPreset::Rubber)
    }
}

impl Material {
    /// Create a new material with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `youngs_modulus` - Young's modulus in Pa
    /// * `poissons_ratio` - Poisson's ratio (0-0.5)
    /// * `density` - Density in kg/m³
    #[must_use]
    pub fn new(youngs_modulus: f64, poissons_ratio: f64, density: f64) -> Self {
        Self {
            youngs_modulus,
            poissons_ratio: poissons_ratio.clamp(0.0, 0.499),
            density,
            damping: 0.01,
            stretch_factor: 1.0,
            bend_factor: 1.0,
            volume_factor: 1.0,
            friction: 0.5,
            compressible: poissons_ratio < 0.49,
        }
    }

    /// Create a material from a preset.
    #[must_use]
    pub const fn preset(preset: MaterialPreset) -> Self {
        match preset {
            MaterialPreset::Rubber => Self {
                youngs_modulus: 1e6,  // 1 MPa (soft rubber)
                poissons_ratio: 0.49, // Nearly incompressible
                density: 1100.0,      // kg/m³
                damping: 0.02,
                stretch_factor: 1.0,
                bend_factor: 1.0,
                volume_factor: 1.0,
                friction: 0.9,
                compressible: false,
            },
            MaterialPreset::Cloth => Self {
                youngs_modulus: 1e5, // 100 kPa (cotton-like)
                poissons_ratio: 0.3,
                density: 300.0, // Light fabric
                damping: 0.05,
                stretch_factor: 1.0,
                bend_factor: 0.1,   // Cloth bends easily
                volume_factor: 0.0, // No volume constraint for 2D
                friction: 0.3,
                compressible: true,
            },
            MaterialPreset::SoftTissue => Self {
                youngs_modulus: 1e4,  // 10 kPa (soft tissue)
                poissons_ratio: 0.45, // Nearly incompressible
                density: 1050.0,      // Similar to water
                damping: 0.1,
                stretch_factor: 1.0,
                bend_factor: 0.5,
                volume_factor: 1.0,
                friction: 0.2,
                compressible: false,
            },
            MaterialPreset::Muscle => Self {
                youngs_modulus: 5e4,  // 50 kPa (relaxed muscle)
                poissons_ratio: 0.49, // Nearly incompressible
                density: 1060.0,
                damping: 0.15,
                stretch_factor: 1.0,
                bend_factor: 0.3,
                volume_factor: 1.0,
                friction: 0.3,
                compressible: false,
            },
            MaterialPreset::Cartilage => Self {
                youngs_modulus: 1e6, // 1 MPa
                poissons_ratio: 0.45,
                density: 1100.0,
                damping: 0.05,
                stretch_factor: 1.0,
                bend_factor: 1.0,
                volume_factor: 1.0,
                friction: 0.01, // Very low friction
                compressible: false,
            },
            MaterialPreset::Gelatin => Self {
                youngs_modulus: 1e3, // 1 kPa (very soft)
                poissons_ratio: 0.49,
                density: 1050.0,
                damping: 0.2,
                stretch_factor: 1.0,
                bend_factor: 0.1,
                volume_factor: 1.0,
                friction: 0.5,
                compressible: false,
            },
            MaterialPreset::Foam => Self {
                youngs_modulus: 1e4, // 10 kPa
                poissons_ratio: 0.2, // Compressible
                density: 50.0,       // Very light
                damping: 0.3,
                stretch_factor: 1.0,
                bend_factor: 0.5,
                volume_factor: 0.5,
                friction: 0.7,
                compressible: true,
            },
            MaterialPreset::Leather => Self {
                youngs_modulus: 1e7, // 10 MPa
                poissons_ratio: 0.4,
                density: 900.0,
                damping: 0.03,
                stretch_factor: 1.0,
                bend_factor: 0.3,
                volume_factor: 0.0,
                friction: 0.6,
                compressible: true,
            },
            MaterialPreset::Rope => Self {
                youngs_modulus: 1e8, // 100 MPa (hemp/cotton)
                poissons_ratio: 0.3,
                density: 800.0,
                damping: 0.02,
                stretch_factor: 1.0,
                bend_factor: 0.01, // Very flexible
                volume_factor: 0.0,
                friction: 0.4,
                compressible: true,
            },
            MaterialPreset::SteelCable => Self {
                youngs_modulus: 2e11, // 200 GPa (steel)
                poissons_ratio: 0.3,
                density: 7800.0,
                damping: 0.001,
                stretch_factor: 1.0,
                bend_factor: 0.001, // Very stiff bending
                volume_factor: 0.0,
                friction: 0.2,
                compressible: true,
            },
            MaterialPreset::Tendon => Self {
                youngs_modulus: 1.2e9, // 1.2 GPa
                poissons_ratio: 0.45,
                density: 1100.0,
                damping: 0.05,
                stretch_factor: 1.0,
                bend_factor: 0.01,
                volume_factor: 0.5,
                friction: 0.1,
                compressible: false,
            },
            MaterialPreset::Paper => Self {
                youngs_modulus: 3e9, // 3 GPa
                poissons_ratio: 0.2,
                density: 700.0,
                damping: 0.1,
                stretch_factor: 1.0,
                bend_factor: 0.001, // Easy to fold
                volume_factor: 0.0,
                friction: 0.4,
                compressible: true,
            },
            MaterialPreset::FlexiblePlastic => Self {
                youngs_modulus: 1e8, // 100 MPa
                poissons_ratio: 0.4,
                density: 1200.0,
                damping: 0.01,
                stretch_factor: 1.0,
                bend_factor: 0.1,
                volume_factor: 0.0,
                friction: 0.3,
                compressible: true,
            },
            MaterialPreset::SoftWood => Self {
                youngs_modulus: 1e10, // 10 GPa
                poissons_ratio: 0.3,
                density: 500.0,
                damping: 0.02,
                stretch_factor: 1.0,
                bend_factor: 1.0,
                volume_factor: 1.0,
                friction: 0.5,
                compressible: true,
            },
        }
    }

    /// Set the damping coefficient.
    #[must_use]
    pub const fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
        self
    }

    /// Set the stretch stiffness factor.
    #[must_use]
    pub const fn with_stretch_factor(mut self, factor: f64) -> Self {
        self.stretch_factor = factor;
        self
    }

    /// Set the bend stiffness factor.
    #[must_use]
    pub const fn with_bend_factor(mut self, factor: f64) -> Self {
        self.bend_factor = factor;
        self
    }

    /// Set the volume preservation factor.
    #[must_use]
    pub const fn with_volume_factor(mut self, factor: f64) -> Self {
        self.volume_factor = factor;
        self
    }

    /// Set the friction coefficient.
    #[must_use]
    pub const fn with_friction(mut self, friction: f64) -> Self {
        self.friction = friction;
        self
    }

    /// Compute the shear modulus G = E / (2(1 + ν)).
    #[must_use]
    pub fn shear_modulus(&self) -> f64 {
        self.youngs_modulus / (2.0 * (1.0 + self.poissons_ratio))
    }

    /// Compute the bulk modulus K = E / (3(1 - 2ν)).
    #[must_use]
    pub fn bulk_modulus(&self) -> f64 {
        // Prevent division by zero for ν = 0.5
        let denom = 3.0 * 2.0f64.mul_add(-self.poissons_ratio, 1.0);
        if denom.abs() < 1e-10 {
            return f64::INFINITY; // Incompressible
        }
        self.youngs_modulus / denom
    }

    /// Compute the first Lamé parameter λ.
    #[must_use]
    pub fn lame_lambda(&self) -> f64 {
        let nu = self.poissons_ratio;
        let denom = (1.0 + nu) * 2.0f64.mul_add(-nu, 1.0);
        if denom.abs() < 1e-10 {
            return f64::INFINITY;
        }
        self.youngs_modulus * nu / denom
    }

    /// Compute the second Lamé parameter μ (same as shear modulus).
    #[must_use]
    pub fn lame_mu(&self) -> f64 {
        self.shear_modulus()
    }

    /// Compute XPBD compliance for distance constraints.
    ///
    /// # Arguments
    ///
    /// * `cross_section_area` - Cross-sectional area of the element
    ///
    /// # Returns
    ///
    /// Compliance α = 1 / (E * A * `stretch_factor`)
    #[must_use]
    pub fn distance_compliance(&self, cross_section_area: f64) -> f64 {
        let stiffness = self.youngs_modulus * cross_section_area * self.stretch_factor;
        if stiffness > 1e-10 {
            1.0 / stiffness
        } else {
            1e10 // Very soft
        }
    }

    /// Compute XPBD compliance for bending constraints.
    ///
    /// # Arguments
    ///
    /// * `second_moment_of_area` - Second moment of area (I)
    ///
    /// # Returns
    ///
    /// Compliance α = 1 / (E * I * `bend_factor`)
    #[must_use]
    pub fn bending_compliance(&self, second_moment_of_area: f64) -> f64 {
        let stiffness = self.youngs_modulus * second_moment_of_area * self.bend_factor;
        if stiffness > 1e-10 {
            1.0 / stiffness
        } else {
            1e10
        }
    }

    /// Compute XPBD compliance for volume constraints.
    ///
    /// # Arguments
    ///
    /// * `rest_volume` - Rest volume of the element
    ///
    /// # Returns
    ///
    /// Compliance α = 1 / (K * V * `volume_factor`)
    #[must_use]
    pub fn volume_compliance(&self, rest_volume: f64) -> f64 {
        if !self.compressible || self.volume_factor < 1e-10 {
            return 0.0; // Incompressible
        }

        let k = self.bulk_modulus();
        let stiffness = k * rest_volume * self.volume_factor;
        if stiffness > 1e-10 && stiffness.is_finite() {
            1.0 / stiffness
        } else if stiffness.is_infinite() || stiffness < 1e-10 {
            0.0 // Incompressible
        } else {
            1e10
        }
    }

    /// Validate that the material parameters are physically valid.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Young's modulus is negative
    /// - Poisson's ratio is outside [0, 0.5]
    /// - Density is negative
    pub fn validate(&self) -> Result<()> {
        if self.youngs_modulus < 0.0 {
            return Err(DeformableError::invalid_material(
                "Young's modulus cannot be negative",
            ));
        }

        if !(0.0..=0.5).contains(&self.poissons_ratio) {
            return Err(DeformableError::invalid_material(
                "Poisson's ratio must be in range [0, 0.5]",
            ));
        }

        if self.density < 0.0 {
            return Err(DeformableError::invalid_material(
                "Density cannot be negative",
            ));
        }

        if self.damping < 0.0 {
            return Err(DeformableError::invalid_material(
                "Damping cannot be negative",
            ));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_material_default() {
        let mat = Material::default();
        assert!(mat.youngs_modulus > 0.0);
        assert!(mat.poissons_ratio >= 0.0 && mat.poissons_ratio <= 0.5);
        assert!(mat.density > 0.0);
    }

    #[test]
    fn test_material_presets() {
        // Just check they don't panic and have valid parameters
        for preset in [
            MaterialPreset::Rubber,
            MaterialPreset::Cloth,
            MaterialPreset::SoftTissue,
            MaterialPreset::Muscle,
            MaterialPreset::Cartilage,
            MaterialPreset::Gelatin,
            MaterialPreset::Foam,
            MaterialPreset::Leather,
            MaterialPreset::Rope,
            MaterialPreset::SteelCable,
            MaterialPreset::Tendon,
            MaterialPreset::Paper,
            MaterialPreset::FlexiblePlastic,
            MaterialPreset::SoftWood,
        ] {
            let mat = Material::preset(preset);
            assert!(
                mat.validate().is_ok(),
                "Preset {:?} failed validation",
                preset
            );
        }
    }

    #[test]
    fn test_material_moduli() {
        let mat = Material::new(1e6, 0.3, 1000.0);

        let g = mat.shear_modulus();
        let k = mat.bulk_modulus();

        // G = E / 2(1+v) = 1e6 / 2.6 ≈ 384615
        assert!((g - 1e6 / 2.6).abs() < 1.0);

        // K = E / 3(1-2v) = 1e6 / 1.2 ≈ 833333
        assert!((k - 1e6 / 1.2).abs() < 1.0);
    }

    #[test]
    fn test_material_compliance() {
        let mat = Material::new(1e6, 0.3, 1000.0);

        // Distance compliance
        let alpha_d = mat.distance_compliance(0.01); // 1 cm² cross section
        // α = 1 / (E * A) = 1 / (1e6 * 0.01) = 1e-4
        assert!((alpha_d - 1e-4).abs() < 1e-10);

        // Bending compliance
        let alpha_b = mat.bending_compliance(1e-8); // Small I
        assert!(alpha_b > 0.0);
    }

    #[test]
    fn test_material_validation() {
        let valid = Material::new(1e6, 0.3, 1000.0);
        assert!(valid.validate().is_ok());

        let invalid_youngs = Material::new(-1.0, 0.3, 1000.0);
        assert!(invalid_youngs.validate().is_err());

        // Note: Material::new clamps poissons_ratio to [0, 0.499]
        // So we create an invalid material directly
        let mut invalid_poisson = Material::new(1e6, 0.3, 1000.0);
        invalid_poisson.poissons_ratio = 0.6;
        assert!(invalid_poisson.validate().is_err());

        let invalid_density = Material::new(1e6, 0.3, -1.0);
        assert!(invalid_density.validate().is_err());
    }

    #[test]
    fn test_material_builder() {
        let mat = Material::preset(MaterialPreset::Rubber)
            .with_damping(0.1)
            .with_stretch_factor(2.0)
            .with_bend_factor(0.5)
            .with_friction(0.8);

        assert_eq!(mat.damping, 0.1);
        assert_eq!(mat.stretch_factor, 2.0);
        assert_eq!(mat.bend_factor, 0.5);
        assert_eq!(mat.friction, 0.8);
    }

    #[test]
    fn test_incompressible_material() {
        // Nearly incompressible (ν close to 0.5)
        let mat = Material::new(1e6, 0.49, 1000.0);

        let k = mat.bulk_modulus();
        // K should be very large for nearly incompressible material
        assert!(k > 1e7);

        // Volume compliance should be small
        let alpha_v = mat.volume_compliance(0.001);
        assert!(alpha_v < 1e-10 || alpha_v == 0.0);
    }
}
