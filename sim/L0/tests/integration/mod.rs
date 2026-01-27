//! Integration tests for the sim-* crate ecosystem.
//!
//! These tests verify end-to-end functionality of the simulation pipeline:
//! - MJCF loading → Model/Data simulation (MuJoCo-aligned API)
//! - URDF loading → Model/Data simulation (via MJCF conversion)
//! - Musculoskeletal dynamics (muscles, tendons)
//! - Sensor readings from simulation state
//! - Phase 7 validation tests for MuJoCo consolidation

pub mod model_data_pipeline;
pub mod musculoskeletal;
pub mod sensors;
pub mod validation;
