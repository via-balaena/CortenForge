//! Lattice structure and infill pattern generation for 3D printing.
//!
//! This crate provides algorithms for generating lattice structures within
//! bounding volumes, suitable for 3D printing lightweight parts with controlled
//! mechanical properties.
//!
//! # Lattice Types
//!
//! - **Cubic**: Simple grid pattern, easy to print
//! - **Octet-Truss**: High strength-to-weight ratio structural lattice
//! - **Gyroid**: TPMS surface, smooth and organic, self-supporting
//! - **Schwarz-P**: TPMS surface variant with different properties
//! - **Diamond**: TPMS surface with good mechanical isotropy
//! - **Voronoi**: Organic, irregular cell structure
//!
//! # Quick Start
//!
//! ```
//! use mesh_lattice::{generate_lattice, LatticeParams, LatticeType};
//! use nalgebra::Point3;
//!
//! // Generate a cubic lattice in a bounding box
//! let params = LatticeParams::cubic(5.0);
//! let bounds = (
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(50.0, 50.0, 50.0),
//! );
//! let result = generate_lattice(&params, bounds).unwrap();
//! println!("Generated lattice with {} vertices", result.vertex_count());
//! ```
//!
//! # Variable Density
//!
//! Lattices can have variable density using [`DensityMap`]:
//!
//! ```
//! use mesh_lattice::{LatticeParams, DensityMap};
//! use nalgebra::Point3;
//!
//! // Gradient density from bottom to top
//! let density = DensityMap::Gradient {
//!     from: Point3::new(0.0, 0.0, 0.0),
//!     from_density: 0.1,
//!     to: Point3::new(0.0, 0.0, 100.0),
//!     to_density: 0.5,
//! };
//! let params = LatticeParams::gyroid(8.0).with_density_map(density);
//! ```
//!
//! # Architecture
//!
//! This is a Layer 0 crate with no Bevy dependencies. It can be used
//! standalone or through the `cortenforge` Bevy SDK.

mod beam;
mod density;
mod error;
mod generate;
mod infill;
mod marching_cubes;
mod params;
mod strut;
mod tpms;
mod types;

pub use beam::{Beam, BeamCap, BeamLatticeData, BeamSet};
pub use density::DensityMap;
pub use error::LatticeError;
pub use generate::generate_lattice;
pub use infill::{generate_infill, InfillParams, InfillResult};
pub use params::LatticeParams;
pub use strut::{combine_struts, estimate_strut_volume, generate_strut, generate_strut_tapered};
pub use tpms::{density_to_threshold, diamond, gyroid, iwp, make_shell, neovius, schwarz_p};
pub use types::{LatticeResult, LatticeType};

/// Re-export marching cubes for advanced usage.
pub mod marching_cubes_algorithm {
    pub use crate::marching_cubes::extract_isosurface;
}
