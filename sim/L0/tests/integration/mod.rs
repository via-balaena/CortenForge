//! Integration tests for the sim-* crate ecosystem.
//!
//! These tests verify end-to-end functionality of the simulation pipeline:
//! - URDF loading → World simulation
//! - MJCF loading → World simulation
//! - MJCF loading → Model/Data simulation (new MuJoCo-aligned API)
//! - Musculoskeletal dynamics (muscles, tendons)
//! - Sensor readings from simulation state
//! - Phase 7 validation tests for MuJoCo consolidation

pub mod mjcf_pipeline;
pub mod model_data_pipeline;
pub mod musculoskeletal;
pub mod sensors;
pub mod urdf_pipeline;
pub mod validation;
