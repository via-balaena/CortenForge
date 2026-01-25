//! Integration tests for the sim-* crate ecosystem.
//!
//! These tests verify end-to-end functionality of the simulation pipeline:
//! - URDF loading → World simulation
//! - MJCF loading → World simulation
//! - Musculoskeletal dynamics (muscles, tendons)
//! - Sensor readings from simulation state

pub mod mjcf_pipeline;
pub mod musculoskeletal;
pub mod sensors;
pub mod urdf_pipeline;
