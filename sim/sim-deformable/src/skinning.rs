//! Skinned mesh support for deformable body visualization.
//!
//! This module provides vertex skinning capabilities for rendering deformable bodies
//! with smooth visual meshes. It implements two skinning algorithms:
//!
//! - **Linear Blend Skinning (LBS)**: Fast, simple, widely supported
//! - **Dual Quaternion Skinning (DQS)**: Better for large rotations, no volume loss
//!
//! # Overview
//!
//! Skinned meshes map a high-resolution visual mesh to a low-resolution physics
//! simulation (bones). Each vertex in the visual mesh is influenced by one or more
//! bones, with weights determining the contribution of each bone's transform.
//!
//! # Linear Blend Skinning (LBS)
//!
//! LBS computes vertex positions as a weighted sum of bone-transformed positions:
//!
//! ```text
//! v' = Σᵢ wᵢ · Mᵢ · M_bind_i⁻¹ · v
//! ```
//!
//! Where:
//! - `v'` is the skinned vertex position
//! - `wᵢ` is the weight for bone i
//! - `Mᵢ` is the current pose of bone i
//! - `M_bind_i` is the bind pose (rest pose) of bone i
//! - `v` is the original vertex position
//!
//! LBS is fast but can exhibit "candy wrapper" artifacts for large rotations.
//!
//! # Dual Quaternion Skinning (DQS)
//!
//! DQS uses dual quaternions to represent rigid transforms, avoiding the volume
//! loss and collapse artifacts of LBS for large rotations:
//!
//! ```text
//! dq = Σᵢ wᵢ · DQ(Mᵢ · M_bind_i⁻¹)
//! dq_normalized = dq / |dq|
//! v' = transform(dq_normalized, v)
//! ```
//!
//! DQS is slightly more expensive but produces better results for joints with
//! large rotation ranges (shoulders, hips, wrists).
//!
//! # Usage
//!
//! ```ignore
//! use sim_deformable::skinning::{
//!     Skeleton, Bone, SkinnedMesh, BoneWeight, SkinningMethod,
//! };
//! use sim_types::Pose;
//! use nalgebra::Point3;
//!
//! // Create a skeleton with two bones (arm)
//! let mut skeleton = Skeleton::new();
//! skeleton.add_root_bone("upper_arm", Pose::identity());
//! skeleton.add_bone("forearm", Some(0), Pose::from_position(Point3::new(0.0, 0.0, -0.3)));
//!
//! // Create a skinned mesh from visual mesh vertices
//! let vertices = vec![/* visual mesh vertices */];
//! let mut skinned_mesh = SkinnedMesh::new(vertices);
//!
//! // Assign bone weights to each vertex
//! skinned_mesh.set_vertex_weights(0, vec![
//!     BoneWeight::new(0, 0.8),  // 80% upper arm
//!     BoneWeight::new(1, 0.2),  // 20% forearm
//! ]);
//!
//! // Update skeleton poses from simulation
//! skeleton.set_bone_pose(0, new_upper_arm_pose);
//! skeleton.set_bone_pose(1, new_forearm_pose);
//!
//! // Compute skinned vertex positions
//! let deformed = skinned_mesh.compute_skinned_positions(&skeleton, SkinningMethod::LinearBlend);
//! ```
//!
//! # Integration with Simulation
//!
//! The skinning module is designed to work with the `sim-deformable` deformable bodies:
//!
//! - `SoftBody` particles become bones in the skeleton
//! - `Cloth` particles become bones for cloth visualization
//! - `CapsuleChain` segments become bones for rope/cable rendering
//!
//! # Layer 0 Crate
//!
//! This module has no rendering dependencies. It computes deformed vertex positions
//! that can be consumed by any rendering system (Bevy, Three.js, Vulkan, etc.).

use nalgebra::{Point3, UnitQuaternion, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::error::{DeformableError, Result};

// Re-export Pose from sim-types for convenience
pub use sim_types::Pose;

/// A bone in a skeleton hierarchy.
///
/// Bones represent rigid transforms that influence vertex positions during skinning.
/// Each bone has a bind pose (the pose at which vertex weights were assigned) and
/// a current pose (updated during simulation).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Bone {
    /// Name of the bone (for debugging and MJCF mapping).
    pub name: String,

    /// Index of the parent bone, or `None` for root bones.
    pub parent: Option<usize>,

    /// The bind pose (rest pose) of this bone in local space.
    ///
    /// This is the pose relative to the parent bone at which vertex weights
    /// were assigned. It's used to compute the inverse bind matrix.
    pub bind_pose: Pose,

    /// The current pose of this bone in local space.
    ///
    /// This is updated during simulation and used to compute deformed vertices.
    pub local_pose: Pose,

    /// Cached world-space pose (computed from parent chain).
    world_pose: Pose,

    /// Cached inverse bind pose in world space.
    inverse_bind_pose: Pose,
}

impl Bone {
    /// Create a new bone with the given name and bind pose.
    #[must_use]
    pub fn new(name: impl Into<String>, bind_pose: Pose) -> Self {
        Self {
            name: name.into(),
            parent: None,
            bind_pose,
            local_pose: bind_pose,
            world_pose: bind_pose,
            inverse_bind_pose: bind_pose.inverse(),
        }
    }

    /// Create a bone with a parent.
    #[must_use]
    pub const fn with_parent(mut self, parent_index: usize) -> Self {
        self.parent = Some(parent_index);
        self
    }

    /// Get the bone's name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the bone's current world-space pose.
    #[must_use]
    pub const fn world_pose(&self) -> &Pose {
        &self.world_pose
    }

    /// Get the inverse bind pose.
    #[must_use]
    pub const fn inverse_bind_pose(&self) -> &Pose {
        &self.inverse_bind_pose
    }

    /// Compute the skinning matrix: `world_pose` * `inverse_bind_pose`.
    ///
    /// This matrix transforms a vertex from bind pose to current pose.
    #[must_use]
    pub fn skinning_transform(&self) -> Pose {
        self.world_pose.compose(&self.inverse_bind_pose)
    }
}

/// A skeleton containing a hierarchy of bones.
///
/// The skeleton manages bone relationships and computes world-space transforms
/// for all bones based on the hierarchy.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Skeleton {
    /// All bones in the skeleton.
    bones: Vec<Bone>,

    /// Indices of root bones (bones without parents).
    roots: Vec<usize>,
}

impl Skeleton {
    /// Create an empty skeleton.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a skeleton with preallocated capacity.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            bones: Vec::with_capacity(capacity),
            roots: Vec::new(),
        }
    }

    /// Get the number of bones in the skeleton.
    #[must_use]
    pub fn num_bones(&self) -> usize {
        self.bones.len()
    }

    /// Check if the skeleton is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bones.is_empty()
    }

    /// Add a root bone to the skeleton.
    ///
    /// Returns the index of the new bone.
    pub fn add_root_bone(&mut self, name: impl Into<String>, bind_pose: Pose) -> usize {
        let index = self.bones.len();
        let mut bone = Bone::new(name, bind_pose);

        // Root bones have world pose = local pose
        bone.world_pose = bind_pose;
        bone.inverse_bind_pose = bind_pose.inverse();

        self.bones.push(bone);
        self.roots.push(index);
        index
    }

    /// Add a bone with a parent to the skeleton.
    ///
    /// # Arguments
    ///
    /// * `name` - Name of the bone
    /// * `parent` - Index of the parent bone, or `None` for a root bone
    /// * `local_bind_pose` - Bind pose relative to the parent
    ///
    /// Returns the index of the new bone.
    ///
    /// # Errors
    ///
    /// Returns an error if the parent index is out of bounds.
    pub fn add_bone(
        &mut self,
        name: impl Into<String>,
        parent: Option<usize>,
        local_bind_pose: Pose,
    ) -> Result<usize> {
        if let Some(parent_idx) = parent {
            if parent_idx >= self.bones.len() {
                return Err(DeformableError::index_out_of_bounds(format!(
                    "Parent bone index {} out of bounds for {} bones",
                    parent_idx,
                    self.bones.len()
                )));
            }
        }

        let index = self.bones.len();
        let mut bone = Bone::new(name, local_bind_pose);

        if let Some(parent_idx) = parent {
            bone.parent = Some(parent_idx);
            // Compute world bind pose
            let parent_world = self.bones[parent_idx].world_pose;
            bone.world_pose = parent_world.compose(&local_bind_pose);
            bone.inverse_bind_pose = bone.world_pose.inverse();
        } else {
            // Root bone
            bone.world_pose = local_bind_pose;
            bone.inverse_bind_pose = local_bind_pose.inverse();
            self.roots.push(index);
        }

        self.bones.push(bone);
        Ok(index)
    }

    /// Get a bone by index.
    #[must_use]
    pub fn bone(&self, index: usize) -> Option<&Bone> {
        self.bones.get(index)
    }

    /// Get a mutable reference to a bone by index.
    #[must_use]
    pub fn bone_mut(&mut self, index: usize) -> Option<&mut Bone> {
        self.bones.get_mut(index)
    }

    /// Find a bone by name.
    #[must_use]
    pub fn find_bone(&self, name: &str) -> Option<usize> {
        self.bones.iter().position(|b| b.name == name)
    }

    /// Get all bones in the skeleton.
    #[must_use]
    pub fn bones(&self) -> &[Bone] {
        &self.bones
    }

    /// Set the local pose of a bone.
    ///
    /// This updates the bone's local pose. Call `update_world_poses()` after
    /// setting all bone poses to propagate changes through the hierarchy.
    pub fn set_bone_local_pose(&mut self, index: usize, pose: Pose) {
        if let Some(bone) = self.bones.get_mut(index) {
            bone.local_pose = pose;
        }
    }

    /// Set the world pose of a bone directly.
    ///
    /// This bypasses the hierarchy and sets the world pose directly.
    /// Useful when bone poses come from simulation particles.
    pub fn set_bone_world_pose(&mut self, index: usize, pose: Pose) {
        if let Some(bone) = self.bones.get_mut(index) {
            bone.world_pose = pose;
        }
    }

    /// Update all world poses based on the bone hierarchy.
    ///
    /// Call this after setting bone local poses to propagate changes.
    pub fn update_world_poses(&mut self) {
        // Process bones in order (parents before children due to add order)
        for i in 0..self.bones.len() {
            let world_pose = if let Some(parent_idx) = self.bones[i].parent {
                let parent_world = self.bones[parent_idx].world_pose;
                parent_world.compose(&self.bones[i].local_pose)
            } else {
                self.bones[i].local_pose
            };
            self.bones[i].world_pose = world_pose;
        }
    }

    /// Reset all bones to their bind pose.
    pub fn reset_to_bind_pose(&mut self) {
        for bone in &mut self.bones {
            bone.local_pose = bone.bind_pose;
        }
        self.update_world_poses();
    }

    /// Get the root bone indices.
    #[must_use]
    pub fn roots(&self) -> &[usize] {
        &self.roots
    }

    /// Get children of a bone.
    #[must_use]
    pub fn children(&self, bone_index: usize) -> Vec<usize> {
        self.bones
            .iter()
            .enumerate()
            .filter_map(|(i, b)| {
                if b.parent == Some(bone_index) {
                    Some(i)
                } else {
                    None
                }
            })
            .collect()
    }
}

/// Weight of a single bone's influence on a vertex.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BoneWeight {
    /// Index of the bone.
    pub bone_index: usize,

    /// Weight of this bone's influence (0.0 to 1.0).
    ///
    /// Weights for all bones influencing a vertex should sum to 1.0.
    pub weight: f64,
}

impl BoneWeight {
    /// Create a new bone weight.
    #[must_use]
    pub const fn new(bone_index: usize, weight: f64) -> Self {
        Self { bone_index, weight }
    }

    /// Check if this weight is effectively zero.
    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.weight.abs() < 1e-10
    }
}

/// Weights for a single vertex.
///
/// Optimized storage for common cases (1-4 bones per vertex).
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VertexWeights {
    /// Bone weights for this vertex.
    ///
    /// Most vertices are influenced by 1-4 bones. Using a small Vec
    /// avoids heap allocation for common cases via small-vec optimization.
    weights: Vec<BoneWeight>,
}

impl VertexWeights {
    /// Create empty vertex weights.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            weights: Vec::new(),
        }
    }

    /// Create vertex weights from a list of bone weights.
    #[must_use]
    pub const fn from_weights(weights: Vec<BoneWeight>) -> Self {
        Self { weights }
    }

    /// Add a bone weight.
    pub fn add_weight(&mut self, bone_index: usize, weight: f64) {
        if weight.abs() > 1e-10 {
            self.weights.push(BoneWeight::new(bone_index, weight));
        }
    }

    /// Get all weights for this vertex.
    #[must_use]
    pub fn weights(&self) -> &[BoneWeight] {
        &self.weights
    }

    /// Check if this vertex has any weights.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.weights.is_empty()
    }

    /// Get the number of bone influences.
    #[must_use]
    pub fn num_influences(&self) -> usize {
        self.weights.len()
    }

    /// Normalize weights so they sum to 1.0.
    pub fn normalize(&mut self) {
        let sum: f64 = self.weights.iter().map(|w| w.weight).sum();
        if sum > 1e-10 {
            for w in &mut self.weights {
                w.weight /= sum;
            }
        }
    }

    /// Check if weights are normalized (sum to ~1.0).
    #[must_use]
    pub fn is_normalized(&self) -> bool {
        let sum: f64 = self.weights.iter().map(|w| w.weight).sum();
        (sum - 1.0).abs() < 1e-6
    }

    /// Remove weights below a threshold and renormalize.
    pub fn prune(&mut self, threshold: f64) {
        self.weights.retain(|w| w.weight >= threshold);
        self.normalize();
    }
}

/// Skinning method to use for vertex deformation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SkinningMethod {
    /// Linear Blend Skinning (LBS).
    ///
    /// Fast and simple, but can exhibit volume loss and "candy wrapper"
    /// artifacts for large rotations.
    #[default]
    LinearBlend,

    /// Dual Quaternion Skinning (DQS).
    ///
    /// Better for large rotations, preserves volume, but slightly more expensive.
    DualQuaternion,
}

/// A dual quaternion for rigid body transforms.
///
/// Dual quaternions represent rotation and translation in a unified way,
/// enabling proper interpolation without volume loss.
///
/// Note: This implementation prioritizes readability of quaternion math over
/// micro-optimizations suggested by clippy. The standard quaternion multiplication
/// formulas are clearer than the suggested `mul_add` versions.
#[derive(Debug, Clone, Copy)]
struct DualQuat {
    /// Real part (rotation).
    real: UnitQuaternion<f64>,
    /// Dual part (translation encoded).
    dual: nalgebra::Quaternion<f64>,
}

#[allow(clippy::suboptimal_flops)] // Keep quaternion math readable over micro-optimized
impl DualQuat {
    /// Create a dual quaternion from a pose.
    fn from_pose(pose: &Pose) -> Self {
        let real = pose.rotation;
        // Dual part: 0.5 * translation_quat * rotation
        let t = nalgebra::Quaternion::new(0.0, pose.position.x, pose.position.y, pose.position.z);
        let dual = nalgebra::Quaternion::new(
            0.5 * (t.w * real.w - t.i * real.i - t.j * real.j - t.k * real.k),
            0.5 * (t.w * real.i + t.i * real.w + t.j * real.k - t.k * real.j),
            0.5 * (t.w * real.j - t.i * real.k + t.j * real.w + t.k * real.i),
            0.5 * (t.w * real.k + t.i * real.j - t.j * real.i + t.k * real.w),
        );
        Self { real, dual }
    }

    /// Create an identity dual quaternion.
    fn identity() -> Self {
        Self {
            real: UnitQuaternion::identity(),
            dual: nalgebra::Quaternion::new(0.0, 0.0, 0.0, 0.0),
        }
    }

    /// Scale the dual quaternion by a scalar.
    fn scale(&self, s: f64) -> Self {
        Self {
            real: UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
                self.real.w * s,
                self.real.i * s,
                self.real.j * s,
                self.real.k * s,
            )),
            dual: nalgebra::Quaternion::new(
                self.dual.w * s,
                self.dual.i * s,
                self.dual.j * s,
                self.dual.k * s,
            ),
        }
    }

    /// Add two dual quaternions.
    fn add(&self, other: &Self) -> Self {
        Self {
            real: UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
                self.real.w + other.real.w,
                self.real.i + other.real.i,
                self.real.j + other.real.j,
                self.real.k + other.real.k,
            )),
            dual: nalgebra::Quaternion::new(
                self.dual.w + other.dual.w,
                self.dual.i + other.dual.i,
                self.dual.j + other.dual.j,
                self.dual.k + other.dual.k,
            ),
        }
    }

    /// Normalize the dual quaternion.
    fn normalize(&self) -> Self {
        let norm = (self.real.w * self.real.w
            + self.real.i * self.real.i
            + self.real.j * self.real.j
            + self.real.k * self.real.k)
            .sqrt();
        if norm < 1e-10 {
            return Self::identity();
        }
        let inv_norm = 1.0 / norm;
        Self {
            real: UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(
                self.real.w * inv_norm,
                self.real.i * inv_norm,
                self.real.j * inv_norm,
                self.real.k * inv_norm,
            )),
            dual: nalgebra::Quaternion::new(
                self.dual.w * inv_norm,
                self.dual.i * inv_norm,
                self.dual.j * inv_norm,
                self.dual.k * inv_norm,
            ),
        }
    }

    /// Transform a point using the dual quaternion.
    fn transform_point(&self, p: &Point3<f64>) -> Point3<f64> {
        // Rotation
        let rotated = self.real * p.coords;

        // Translation: 2 * dual * conj(real)
        let conj_real =
            nalgebra::Quaternion::new(self.real.w, -self.real.i, -self.real.j, -self.real.k);
        let t_quat = nalgebra::Quaternion::new(
            2.0 * (self.dual.w * conj_real.w
                - self.dual.i * conj_real.i
                - self.dual.j * conj_real.j
                - self.dual.k * conj_real.k),
            2.0 * (self.dual.w * conj_real.i
                + self.dual.i * conj_real.w
                + self.dual.j * conj_real.k
                - self.dual.k * conj_real.j),
            2.0 * (self.dual.w * conj_real.j - self.dual.i * conj_real.k
                + self.dual.j * conj_real.w
                + self.dual.k * conj_real.i),
            2.0 * (self.dual.w * conj_real.k + self.dual.i * conj_real.j
                - self.dual.j * conj_real.i
                + self.dual.k * conj_real.w),
        );

        Point3::from(rotated + Vector3::new(t_quat.i, t_quat.j, t_quat.k))
    }
}

/// A skinned mesh with vertex weights and skinning computation.
///
/// The skinned mesh stores the bind-pose vertices and per-vertex bone weights.
/// Given a skeleton with current poses, it computes deformed vertex positions.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SkinnedMesh {
    /// Name of the skinned mesh.
    name: String,

    /// Bind-pose vertex positions.
    bind_vertices: Vec<Point3<f64>>,

    /// Bind-pose vertex normals (optional).
    bind_normals: Option<Vec<Vector3<f64>>>,

    /// Per-vertex bone weights.
    vertex_weights: Vec<VertexWeights>,

    /// Scratch buffer for skinned positions (avoids allocation).
    #[cfg_attr(feature = "serde", serde(skip))]
    skinned_positions: Vec<Point3<f64>>,

    /// Scratch buffer for skinned normals.
    #[cfg_attr(feature = "serde", serde(skip))]
    skinned_normals: Option<Vec<Vector3<f64>>>,
}

impl SkinnedMesh {
    /// Create a new skinned mesh from bind-pose vertices.
    #[must_use]
    pub fn new(name: impl Into<String>, bind_vertices: Vec<Point3<f64>>) -> Self {
        let num_vertices = bind_vertices.len();
        Self {
            name: name.into(),
            bind_vertices,
            bind_normals: None,
            vertex_weights: vec![VertexWeights::new(); num_vertices],
            skinned_positions: Vec::new(),
            skinned_normals: None,
        }
    }

    /// Create a skinned mesh with normals.
    #[must_use]
    pub fn with_normals(
        name: impl Into<String>,
        bind_vertices: Vec<Point3<f64>>,
        bind_normals: Vec<Vector3<f64>>,
    ) -> Self {
        let num_vertices = bind_vertices.len();
        Self {
            name: name.into(),
            bind_vertices,
            bind_normals: Some(bind_normals),
            vertex_weights: vec![VertexWeights::new(); num_vertices],
            skinned_positions: Vec::new(),
            skinned_normals: None,
        }
    }

    /// Get the mesh name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the number of vertices.
    #[must_use]
    pub fn num_vertices(&self) -> usize {
        self.bind_vertices.len()
    }

    /// Get the bind-pose vertices.
    #[must_use]
    pub fn bind_vertices(&self) -> &[Point3<f64>] {
        &self.bind_vertices
    }

    /// Get the bind-pose normals.
    #[must_use]
    pub fn bind_normals(&self) -> Option<&[Vector3<f64>]> {
        self.bind_normals.as_deref()
    }

    /// Set the weights for a single vertex.
    ///
    /// # Errors
    ///
    /// Returns an error if the vertex index is out of bounds.
    pub fn set_vertex_weights(
        &mut self,
        vertex_index: usize,
        weights: Vec<BoneWeight>,
    ) -> Result<()> {
        if vertex_index >= self.bind_vertices.len() {
            return Err(DeformableError::index_out_of_bounds(format!(
                "Vertex index {} out of bounds for {} vertices",
                vertex_index,
                self.bind_vertices.len()
            )));
        }
        self.vertex_weights[vertex_index] = VertexWeights::from_weights(weights);
        Ok(())
    }

    /// Add a bone weight to a vertex.
    ///
    /// # Errors
    ///
    /// Returns an error if the vertex index is out of bounds.
    pub fn add_vertex_weight(
        &mut self,
        vertex_index: usize,
        bone_index: usize,
        weight: f64,
    ) -> Result<()> {
        if vertex_index >= self.bind_vertices.len() {
            return Err(DeformableError::index_out_of_bounds(format!(
                "Vertex index {} out of bounds for {} vertices",
                vertex_index,
                self.bind_vertices.len()
            )));
        }
        self.vertex_weights[vertex_index].add_weight(bone_index, weight);
        Ok(())
    }

    /// Get the weights for a vertex.
    #[must_use]
    pub fn vertex_weights(&self, vertex_index: usize) -> Option<&VertexWeights> {
        self.vertex_weights.get(vertex_index)
    }

    /// Normalize all vertex weights.
    pub fn normalize_all_weights(&mut self) {
        for weights in &mut self.vertex_weights {
            weights.normalize();
        }
    }

    /// Validate that all weights are properly normalized and reference valid bones.
    ///
    /// # Errors
    ///
    /// Returns an error if any weights are invalid.
    pub fn validate(&self, skeleton: &Skeleton) -> Result<()> {
        for (i, weights) in self.vertex_weights.iter().enumerate() {
            // Check bone indices
            for w in weights.weights() {
                if w.bone_index >= skeleton.num_bones() {
                    return Err(DeformableError::index_out_of_bounds(format!(
                        "Vertex {} references bone {} but skeleton only has {} bones",
                        i,
                        w.bone_index,
                        skeleton.num_bones()
                    )));
                }
            }

            // Check normalization (allow unweighted vertices)
            if !weights.is_empty() && !weights.is_normalized() {
                return Err(DeformableError::invalid_config(format!(
                    "Vertex {} weights not normalized (sum: {})",
                    i,
                    weights.weights().iter().map(|w| w.weight).sum::<f64>()
                )));
            }
        }
        Ok(())
    }

    /// Compute skinned vertex positions using Linear Blend Skinning.
    fn compute_lbs(&mut self, skeleton: &Skeleton) {
        self.skinned_positions.clear();
        self.skinned_positions.reserve(self.bind_vertices.len());

        for (i, bind_pos) in self.bind_vertices.iter().enumerate() {
            let weights = &self.vertex_weights[i];

            if weights.is_empty() {
                // No weights - keep original position
                self.skinned_positions.push(*bind_pos);
                continue;
            }

            let mut skinned = Vector3::zeros();
            for w in weights.weights() {
                if let Some(bone) = skeleton.bone(w.bone_index) {
                    let transform = bone.skinning_transform();
                    let transformed = transform.transform_point(bind_pos);
                    skinned += transformed.coords * w.weight;
                }
            }
            self.skinned_positions.push(Point3::from(skinned));
        }

        // Skin normals if present
        if let Some(bind_normals) = &self.bind_normals {
            let mut skinned_normals = Vec::with_capacity(bind_normals.len());

            for (i, bind_normal) in bind_normals.iter().enumerate() {
                let weights = &self.vertex_weights[i];

                if weights.is_empty() {
                    skinned_normals.push(*bind_normal);
                    continue;
                }

                let mut skinned = Vector3::zeros();
                for w in weights.weights() {
                    if let Some(bone) = skeleton.bone(w.bone_index) {
                        let transform = bone.skinning_transform();
                        let transformed = transform.transform_vector(bind_normal);
                        skinned += transformed * w.weight;
                    }
                }

                // Normalize the result
                let len = skinned.norm();
                if len > 1e-10 {
                    skinned_normals.push(skinned / len);
                } else {
                    skinned_normals.push(*bind_normal);
                }
            }

            self.skinned_normals = Some(skinned_normals);
        }
    }

    /// Compute skinned vertex positions using Dual Quaternion Skinning.
    fn compute_dqs(&mut self, skeleton: &Skeleton) {
        self.skinned_positions.clear();
        self.skinned_positions.reserve(self.bind_vertices.len());

        for (i, bind_pos) in self.bind_vertices.iter().enumerate() {
            let weights = &self.vertex_weights[i];

            if weights.is_empty() {
                self.skinned_positions.push(*bind_pos);
                continue;
            }

            // Blend dual quaternions
            let mut blended = DualQuat::identity();
            let mut first = true;

            for w in weights.weights() {
                if let Some(bone) = skeleton.bone(w.bone_index) {
                    let transform = bone.skinning_transform();
                    let dq = DualQuat::from_pose(&transform);

                    if first {
                        blended = dq.scale(w.weight);
                        first = false;
                    } else {
                        // Ensure shortest path (flip if dot product negative)
                        #[allow(clippy::suboptimal_flops)]
                        let dot = blended.real.w * dq.real.w
                            + blended.real.i * dq.real.i
                            + blended.real.j * dq.real.j
                            + blended.real.k * dq.real.k;

                        let scaled = if dot < 0.0 {
                            dq.scale(-w.weight)
                        } else {
                            dq.scale(w.weight)
                        };
                        blended = blended.add(&scaled);
                    }
                }
            }

            // Normalize and transform
            let normalized = blended.normalize();
            self.skinned_positions
                .push(normalized.transform_point(bind_pos));
        }

        // DQS for normals (use rotation part only)
        if let Some(bind_normals) = &self.bind_normals {
            let mut skinned_normals = Vec::with_capacity(bind_normals.len());

            for (i, bind_normal) in bind_normals.iter().enumerate() {
                let weights = &self.vertex_weights[i];

                if weights.is_empty() {
                    skinned_normals.push(*bind_normal);
                    continue;
                }

                // Blend rotations for normal transformation
                let mut blended_rot = UnitQuaternion::identity();
                let mut total_weight = 0.0;

                for w in weights.weights() {
                    if let Some(bone) = skeleton.bone(w.bone_index) {
                        let transform = bone.skinning_transform();
                        // SLERP-like blending for rotations
                        if total_weight == 0.0 {
                            blended_rot = transform.rotation;
                        } else {
                            let t = w.weight / (total_weight + w.weight);
                            blended_rot = blended_rot.slerp(&transform.rotation, t);
                        }
                        total_weight += w.weight;
                    }
                }

                let transformed = blended_rot * bind_normal;
                let len = transformed.norm();
                if len > 1e-10 {
                    skinned_normals.push(transformed / len);
                } else {
                    skinned_normals.push(*bind_normal);
                }
            }

            self.skinned_normals = Some(skinned_normals);
        }
    }

    /// Compute skinned vertex positions.
    ///
    /// Returns a reference to the internal buffer of skinned positions.
    pub fn compute_skinned_positions(
        &mut self,
        skeleton: &Skeleton,
        method: SkinningMethod,
    ) -> &[Point3<f64>] {
        match method {
            SkinningMethod::LinearBlend => self.compute_lbs(skeleton),
            SkinningMethod::DualQuaternion => self.compute_dqs(skeleton),
        }
        &self.skinned_positions
    }

    /// Get the last computed skinned positions.
    ///
    /// Returns `None` if `compute_skinned_positions` hasn't been called yet.
    #[must_use]
    pub fn skinned_positions(&self) -> &[Point3<f64>] {
        &self.skinned_positions
    }

    /// Get the last computed skinned normals.
    #[must_use]
    pub fn skinned_normals(&self) -> Option<&[Vector3<f64>]> {
        self.skinned_normals.as_deref()
    }
}

/// Result of skinning computation, containing deformed positions and normals.
#[derive(Debug, Clone)]
pub struct SkinningResult {
    /// Deformed vertex positions.
    pub positions: Vec<Point3<f64>>,
    /// Deformed vertex normals (if available).
    pub normals: Option<Vec<Vector3<f64>>>,
}

impl SkinnedMesh {
    /// Compute and return owned skinning result.
    ///
    /// Unlike `compute_skinned_positions`, this returns owned data suitable
    /// for passing to external rendering systems.
    #[must_use]
    pub fn compute_skinning_result(
        &mut self,
        skeleton: &Skeleton,
        method: SkinningMethod,
    ) -> SkinningResult {
        self.compute_skinned_positions(skeleton, method);
        SkinningResult {
            positions: self.skinned_positions.clone(),
            normals: self.skinned_normals.clone(),
        }
    }
}

/// Builder for creating skinned meshes from deformable bodies.
///
/// This provides convenient methods for creating skinned mesh representations
/// of `SoftBody`, `Cloth`, and `CapsuleChain` objects.
#[derive(Debug)]
pub struct SkinnedMeshBuilder {
    name: String,
    vertices: Vec<Point3<f64>>,
    normals: Option<Vec<Vector3<f64>>>,
    weights: Vec<VertexWeights>,
}

impl SkinnedMeshBuilder {
    /// Create a new builder with the given name.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            vertices: Vec::new(),
            normals: None,
            weights: Vec::new(),
        }
    }

    /// Set the bind-pose vertices.
    #[must_use]
    pub fn with_vertices(mut self, vertices: Vec<Point3<f64>>) -> Self {
        self.weights = vec![VertexWeights::new(); vertices.len()];
        self.vertices = vertices;
        self
    }

    /// Set the bind-pose normals.
    #[must_use]
    pub fn with_normals(mut self, normals: Vec<Vector3<f64>>) -> Self {
        self.normals = Some(normals);
        self
    }

    /// Set vertex weights from a flat list of `(vertex_index, bone_index, weight)` tuples.
    #[must_use]
    pub fn with_weights(mut self, weights: &[(usize, usize, f64)]) -> Self {
        for &(vertex_idx, bone_idx, weight) in weights {
            if vertex_idx < self.weights.len() {
                self.weights[vertex_idx].add_weight(bone_idx, weight);
            }
        }
        self
    }

    /// Assign all vertices to a single bone with weight 1.0.
    ///
    /// Useful for rigid attachments.
    #[must_use]
    pub fn with_rigid_binding(mut self, bone_index: usize) -> Self {
        for weights in &mut self.weights {
            *weights = VertexWeights::from_weights(vec![BoneWeight::new(bone_index, 1.0)]);
        }
        self
    }

    /// Assign vertices to bones based on nearest bone position.
    ///
    /// Each vertex is assigned to the closest bone with weight 1.0.
    #[must_use]
    pub fn with_nearest_bone_binding(mut self, bone_positions: &[Point3<f64>]) -> Self {
        for (i, vertex) in self.vertices.iter().enumerate() {
            let mut nearest_bone = 0;
            let mut nearest_dist = f64::MAX;

            for (bone_idx, bone_pos) in bone_positions.iter().enumerate() {
                let dist = (vertex - bone_pos).norm();
                if dist < nearest_dist {
                    nearest_dist = dist;
                    nearest_bone = bone_idx;
                }
            }

            self.weights[i] = VertexWeights::from_weights(vec![BoneWeight::new(nearest_bone, 1.0)]);
        }
        self
    }

    /// Assign vertices to bones using distance-based weighting.
    ///
    /// Each vertex is influenced by bones within `max_distance`, with weights
    /// inversely proportional to distance.
    ///
    /// # Arguments
    ///
    /// * `bone_positions` - World positions of bones
    /// * `max_distance` - Maximum distance for bone influence
    /// * `max_influences` - Maximum number of bones per vertex
    #[must_use]
    pub fn with_distance_weighting(
        mut self,
        bone_positions: &[Point3<f64>],
        max_distance: f64,
        max_influences: usize,
    ) -> Self {
        for (i, vertex) in self.vertices.iter().enumerate() {
            let mut influences: Vec<(usize, f64)> = bone_positions
                .iter()
                .enumerate()
                .filter_map(|(bone_idx, bone_pos)| {
                    let dist = (vertex - bone_pos).norm();
                    if dist < max_distance {
                        // Inverse distance weighting
                        Some((bone_idx, 1.0 / (dist + 0.001)))
                    } else {
                        None
                    }
                })
                .collect();

            // Sort by weight (descending) and take top influences
            influences.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
            influences.truncate(max_influences);

            // Normalize
            let total: f64 = influences.iter().map(|(_, w)| w).sum();
            if total > 1e-10 {
                let weights: Vec<BoneWeight> = influences
                    .iter()
                    .map(|(bone_idx, w)| BoneWeight::new(*bone_idx, w / total))
                    .collect();
                self.weights[i] = VertexWeights::from_weights(weights);
            }
        }
        self
    }

    /// Build the skinned mesh.
    #[must_use]
    pub fn build(mut self) -> SkinnedMesh {
        // Normalize all weights
        for weights in &mut self.weights {
            weights.normalize();
        }

        let mut mesh = if let Some(normals) = self.normals {
            SkinnedMesh::with_normals(self.name, self.vertices, normals)
        } else {
            SkinnedMesh::new(self.name, self.vertices)
        };

        mesh.vertex_weights = self.weights;
        mesh
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn create_simple_skeleton() -> Skeleton {
        let mut skeleton = Skeleton::new();

        // Root bone at origin
        skeleton.add_root_bone("root", Pose::identity());

        // Child bone offset by 1 unit in Z
        skeleton
            .add_bone(
                "child",
                Some(0),
                Pose::from_position(Point3::new(0.0, 0.0, 1.0)),
            )
            .unwrap();

        skeleton
    }

    #[test]
    fn test_skeleton_hierarchy() {
        let skeleton = create_simple_skeleton();

        assert_eq!(skeleton.num_bones(), 2);
        assert_eq!(skeleton.bone(0).unwrap().name(), "root");
        assert_eq!(skeleton.bone(1).unwrap().name(), "child");
        assert_eq!(skeleton.bone(1).unwrap().parent, Some(0));

        // Child world pose should be offset
        let child_world = skeleton.bone(1).unwrap().world_pose();
        assert_relative_eq!(child_world.position.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_skeleton_find_bone() {
        let skeleton = create_simple_skeleton();

        assert_eq!(skeleton.find_bone("root"), Some(0));
        assert_eq!(skeleton.find_bone("child"), Some(1));
        assert_eq!(skeleton.find_bone("nonexistent"), None);
    }

    #[test]
    fn test_skeleton_pose_update() {
        // Create a skeleton with child offset along X axis
        let mut skeleton = Skeleton::new();
        skeleton.add_root_bone("root", Pose::identity());
        skeleton
            .add_bone(
                "child",
                Some(0),
                Pose::from_position(Point3::new(1.0, 0.0, 0.0)), // Offset along X
            )
            .unwrap();

        // Rotate root by 90 degrees around Z
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        skeleton.set_bone_local_pose(0, Pose::from_position_rotation(Point3::origin(), rotation));
        skeleton.update_world_poses();

        // Child at (1, 0, 0) rotated 90 degrees around Z should be at (0, 1, 0)
        let child_world = skeleton.bone(1).unwrap().world_pose();
        assert_relative_eq!(child_world.position.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(child_world.position.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(child_world.position.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_bone_weight() {
        let w = BoneWeight::new(0, 0.5);
        assert_eq!(w.bone_index, 0);
        assert_eq!(w.weight, 0.5);
        assert!(!w.is_zero());

        let zero_w = BoneWeight::new(0, 0.0);
        assert!(zero_w.is_zero());
    }

    #[test]
    fn test_vertex_weights_normalization() {
        let mut weights = VertexWeights::new();
        weights.add_weight(0, 1.0);
        weights.add_weight(1, 1.0);

        assert!(!weights.is_normalized());

        weights.normalize();

        assert!(weights.is_normalized());
        assert_relative_eq!(weights.weights()[0].weight, 0.5, epsilon = 1e-10);
        assert_relative_eq!(weights.weights()[1].weight, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_skinned_mesh_basic() {
        let skeleton = create_simple_skeleton();

        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.5),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let mut mesh = SkinnedMesh::new("test", vertices);

        // Vertex 0: 100% root
        mesh.set_vertex_weights(0, vec![BoneWeight::new(0, 1.0)])
            .unwrap();

        // Vertex 1: 50% root, 50% child
        mesh.set_vertex_weights(1, vec![BoneWeight::new(0, 0.5), BoneWeight::new(1, 0.5)])
            .unwrap();

        // Vertex 2: 100% child
        mesh.set_vertex_weights(2, vec![BoneWeight::new(1, 1.0)])
            .unwrap();

        // Validate
        assert!(mesh.validate(&skeleton).is_ok());

        // Compute skinning (identity pose - should match bind pose)
        let skinned = mesh.compute_skinned_positions(&skeleton, SkinningMethod::LinearBlend);

        assert_relative_eq!(
            skinned[0].coords,
            Point3::new(0.0, 0.0, 0.0).coords,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            skinned[1].coords,
            Point3::new(0.0, 0.0, 0.5).coords,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            skinned[2].coords,
            Point3::new(0.0, 0.0, 1.0).coords,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_lbs_vs_dqs() {
        let mut skeleton = create_simple_skeleton();

        // Rotate child bone significantly
        let rotation = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        skeleton.set_bone_local_pose(
            1,
            Pose::from_position_rotation(Point3::new(0.0, 0.0, 1.0), rotation),
        );
        skeleton.update_world_poses();

        let vertices = vec![Point3::new(0.0, 0.0, 1.0)];
        let mut mesh = SkinnedMesh::new("test", vertices);
        mesh.set_vertex_weights(0, vec![BoneWeight::new(1, 1.0)])
            .unwrap();

        // Both methods should produce similar results for single-bone weighting
        let lbs = mesh.compute_skinned_positions(&skeleton, SkinningMethod::LinearBlend);
        let lbs_pos = lbs[0];

        let dqs = mesh.compute_skinned_positions(&skeleton, SkinningMethod::DualQuaternion);
        let dqs_pos = dqs[0];

        // For single-bone influence, LBS and DQS should be identical
        assert_relative_eq!(lbs_pos.coords, dqs_pos.coords, epsilon = 1e-6);
    }

    #[test]
    fn test_skinned_mesh_builder() {
        let skeleton = create_simple_skeleton();
        let bone_positions = vec![Point3::origin(), Point3::new(0.0, 0.0, 1.0)];

        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.5),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let mesh = SkinnedMeshBuilder::new("test")
            .with_vertices(vertices)
            .with_nearest_bone_binding(&bone_positions)
            .build();

        assert!(mesh.validate(&skeleton).is_ok());

        // Check bindings
        assert_eq!(mesh.vertex_weights(0).unwrap().weights()[0].bone_index, 0);
        assert_eq!(mesh.vertex_weights(2).unwrap().weights()[0].bone_index, 1);
    }

    #[test]
    fn test_skinning_with_normals() {
        let skeleton = create_simple_skeleton();

        let vertices = vec![Point3::new(0.0, 0.0, 0.5)];
        let normals = vec![Vector3::new(1.0, 0.0, 0.0)];

        let mut mesh = SkinnedMesh::with_normals("test", vertices, normals);
        mesh.set_vertex_weights(0, vec![BoneWeight::new(0, 1.0)])
            .unwrap();

        mesh.compute_skinned_positions(&skeleton, SkinningMethod::LinearBlend);

        let skinned_normals = mesh.skinned_normals().unwrap();
        assert_eq!(skinned_normals.len(), 1);
        assert_relative_eq!(skinned_normals[0].x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_dual_quaternion_basic() {
        let pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
        let dq = DualQuat::from_pose(&pose);

        let p = Point3::origin();
        let transformed = dq.transform_point(&p);

        assert_relative_eq!(transformed.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_skeleton_reset_to_bind() {
        let mut skeleton = create_simple_skeleton();

        // Modify pose
        skeleton.set_bone_local_pose(0, Pose::from_position(Point3::new(5.0, 0.0, 0.0)));
        skeleton.update_world_poses();

        // Reset
        skeleton.reset_to_bind_pose();

        // Should be back to original
        let root_world = skeleton.bone(0).unwrap().world_pose();
        assert_relative_eq!(
            root_world.position.coords,
            Point3::origin().coords,
            epsilon = 1e-10
        );
    }
}
