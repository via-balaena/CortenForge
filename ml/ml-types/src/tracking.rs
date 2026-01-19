//! Object tracking types for multi-object tracking (MOT).

use serde::{Deserialize, Serialize};

use crate::BoundingBox;

/// A unique identifier for a tracked object.
///
/// Track IDs persist across frames to identify the same object over time.
/// IDs are unique within a tracking session but may be reused across sessions.
///
/// # Example
///
/// ```
/// use ml_types::TrackId;
///
/// let id = TrackId::new(42);
/// assert_eq!(id.as_u64(), 42);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TrackId(u64);

impl TrackId {
    /// Creates a new track ID.
    #[must_use]
    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    /// Returns the underlying ID value.
    #[must_use]
    pub const fn as_u64(&self) -> u64 {
        self.0
    }
}

impl From<u64> for TrackId {
    fn from(id: u64) -> Self {
        Self::new(id)
    }
}

impl From<TrackId> for u64 {
    fn from(id: TrackId) -> Self {
        id.0
    }
}

impl std::fmt::Display for TrackId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Track({})", self.0)
    }
}

/// The lifecycle state of a tracked object.
///
/// Tracks transition through states based on detection matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum TrackLifecycle {
    /// Track has been created but not yet confirmed.
    #[default]
    Tentative,
    /// Track has been confirmed (matched for N consecutive frames).
    Confirmed,
    /// Track has been lost (no match for recent frames).
    Lost,
    /// Track has been deleted (lost for too long).
    Deleted,
}

impl TrackLifecycle {
    /// Returns `true` if the track is active (tentative or confirmed).
    #[must_use]
    pub const fn is_active(&self) -> bool {
        matches!(self, Self::Tentative | Self::Confirmed)
    }

    /// Returns `true` if the track should be output (confirmed only).
    #[must_use]
    pub const fn is_confirmed(&self) -> bool {
        matches!(self, Self::Confirmed)
    }
}

/// Complete state of a tracked object.
///
/// Contains all information needed to maintain a track across frames,
/// including position, velocity estimates, and metadata.
///
/// # Example
///
/// ```
/// use ml_types::{TrackingState, TrackId, TrackLifecycle, BoundingBox};
///
/// let state = TrackingState::new(
///     TrackId::new(1),
///     BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0),
///     0,
/// );
///
/// assert_eq!(state.track_id().as_u64(), 1);
/// assert!(state.lifecycle().is_active());
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrackingState {
    /// Unique track identifier.
    track_id: TrackId,

    /// Current bounding box estimate.
    bbox: BoundingBox,

    /// Predicted velocity (normalized units per frame): `[dx, dy, dw, dh]`.
    velocity: [f32; 4],

    /// Track lifecycle state.
    lifecycle: TrackLifecycle,

    /// Frame number when track was created.
    created_frame: u64,

    /// Frame number of last successful detection match.
    last_matched_frame: u64,

    /// Number of consecutive frames without a match.
    frames_since_match: u32,

    /// Total number of successful matches.
    total_matches: u32,

    /// Class ID of the tracked object.
    class_id: u32,

    /// Optional class name.
    class_name: Option<String>,

    /// Accumulated confidence from detections.
    confidence_sum: f32,
}

impl TrackingState {
    /// Creates a new tracking state from an initial detection.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(track_id: TrackId, bbox: BoundingBox, frame: u64) -> Self {
        Self {
            track_id,
            class_id: bbox.class_id,
            bbox,
            velocity: [0.0; 4],
            lifecycle: TrackLifecycle::Tentative,
            created_frame: frame,
            last_matched_frame: frame,
            frames_since_match: 0,
            total_matches: 1,
            class_name: None,
            confidence_sum: bbox.confidence,
        }
    }

    /// Returns the track ID.
    #[must_use]
    pub const fn track_id(&self) -> TrackId {
        self.track_id
    }

    /// Returns the current bounding box estimate.
    #[must_use]
    pub const fn bbox(&self) -> &BoundingBox {
        &self.bbox
    }

    /// Returns the estimated velocity.
    #[must_use]
    pub const fn velocity(&self) -> [f32; 4] {
        self.velocity
    }

    /// Returns the lifecycle state.
    #[must_use]
    pub const fn lifecycle(&self) -> TrackLifecycle {
        self.lifecycle
    }

    /// Returns the frame when this track was created.
    #[must_use]
    pub const fn created_frame(&self) -> u64 {
        self.created_frame
    }

    /// Returns the frame of the last detection match.
    #[must_use]
    pub const fn last_matched_frame(&self) -> u64 {
        self.last_matched_frame
    }

    /// Returns the number of frames since the last match.
    #[must_use]
    pub const fn frames_since_match(&self) -> u32 {
        self.frames_since_match
    }

    /// Returns the total number of matches.
    #[must_use]
    pub const fn total_matches(&self) -> u32 {
        self.total_matches
    }

    /// Returns the class ID.
    #[must_use]
    pub const fn class_id(&self) -> u32 {
        self.class_id
    }

    /// Returns the class name if set.
    #[must_use]
    pub fn class_name(&self) -> Option<&str> {
        self.class_name.as_deref()
    }

    /// Sets the class name.
    pub fn set_class_name(&mut self, name: impl Into<String>) {
        self.class_name = Some(name.into());
    }

    /// Returns the average detection confidence.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn average_confidence(&self) -> f32 {
        if self.total_matches == 0 {
            0.0
        } else {
            self.confidence_sum / self.total_matches as f32
        }
    }

    /// Returns the track age in frames.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn age(&self, current_frame: u64) -> u64 {
        current_frame.saturating_sub(self.created_frame)
    }

    /// Predicts the bounding box for the next frame using velocity.
    #[must_use]
    pub fn predict(&self) -> BoundingBox {
        let [dx, dy, dw, dh] = self.velocity;
        BoundingBox {
            x0: self.bbox.x0 + dx,
            y0: self.bbox.y0 + dy,
            x1: self.bbox.x1 + dx + dw,
            y1: self.bbox.y1 + dy + dh,
            confidence: self.bbox.confidence * 0.9, // Decay confidence
            class_id: self.bbox.class_id,
        }
    }

    /// Updates the track with a matched detection.
    pub fn update_matched(&mut self, detection: &BoundingBox, frame: u64, alpha: f32) {
        // Update velocity estimate (exponential moving average)
        let dx = detection.x0 - self.bbox.x0;
        let dy = detection.y0 - self.bbox.y0;
        let dw = detection.width() - self.bbox.width();
        let dh = detection.height() - self.bbox.height();

        self.velocity[0] = alpha.mul_add(dx - self.velocity[0], self.velocity[0]);
        self.velocity[1] = alpha.mul_add(dy - self.velocity[1], self.velocity[1]);
        self.velocity[2] = alpha.mul_add(dw - self.velocity[2], self.velocity[2]);
        self.velocity[3] = alpha.mul_add(dh - self.velocity[3], self.velocity[3]);

        // Update bounding box
        self.bbox = *detection;

        // Update tracking state
        self.last_matched_frame = frame;
        self.frames_since_match = 0;
        self.total_matches += 1;
        self.confidence_sum += detection.confidence;

        // Confirm track after enough matches, or re-confirm if Lost
        if (self.lifecycle == TrackLifecycle::Tentative && self.total_matches >= 3)
            || self.lifecycle == TrackLifecycle::Lost
        {
            self.lifecycle = TrackLifecycle::Confirmed;
        }
    }

    /// Updates the track when no detection was matched.
    pub fn update_unmatched(&mut self, max_age: u32) {
        self.frames_since_match += 1;

        // Use predicted position
        self.bbox = self.predict();

        // Update lifecycle
        if self.frames_since_match > max_age {
            self.lifecycle = TrackLifecycle::Deleted;
        } else if self.lifecycle == TrackLifecycle::Confirmed {
            self.lifecycle = TrackLifecycle::Lost;
        } else if self.lifecycle == TrackLifecycle::Tentative {
            self.lifecycle = TrackLifecycle::Deleted;
        }
    }

    /// Returns `true` if this track should be deleted.
    #[must_use]
    pub const fn should_delete(&self) -> bool {
        matches!(self.lifecycle, TrackLifecycle::Deleted)
    }

    /// Returns `true` if this track should be output.
    #[must_use]
    pub const fn should_output(&self) -> bool {
        self.lifecycle.is_confirmed()
    }
}

/// A collection of tracks for multi-object tracking.
///
/// Manages track lifecycle, ID assignment, and provides
/// query methods for active tracks.
///
/// # Example
///
/// ```
/// use ml_types::{TrackRegistry, BoundingBox};
///
/// let mut registry = TrackRegistry::new();
///
/// let bbox = BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0);
/// let track_id = registry.create_track(bbox, 0);
///
/// assert_eq!(registry.active_count(), 1);
/// ```
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TrackRegistry {
    /// All tracks (including deleted ones for history).
    tracks: Vec<TrackingState>,
    /// Next track ID to assign.
    next_id: u64,
    /// Maximum frames without match before deletion.
    max_age: u32,
    /// Minimum matches to confirm a track.
    min_hits: u32,
}

impl TrackRegistry {
    /// Creates a new track registry with default parameters.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new() -> Self {
        Self {
            tracks: Vec::new(),
            next_id: 1,
            max_age: 30,
            min_hits: 3,
        }
    }

    /// Creates a track registry with custom parameters.
    #[must_use]
    pub const fn with_params(max_age: u32, min_hits: u32) -> Self {
        Self {
            tracks: Vec::new(),
            next_id: 1,
            max_age,
            min_hits,
        }
    }

    /// Creates a new track from a detection.
    ///
    /// Returns the assigned track ID.
    pub fn create_track(&mut self, detection: BoundingBox, frame: u64) -> TrackId {
        let track_id = TrackId::new(self.next_id);
        self.next_id += 1;

        let state = TrackingState::new(track_id, detection, frame);
        self.tracks.push(state);

        track_id
    }

    /// Returns the total number of tracks (including deleted).
    #[must_use]
    pub fn len(&self) -> usize {
        self.tracks.len()
    }

    /// Returns `true` if there are no tracks.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tracks.is_empty()
    }

    /// Returns the number of active tracks.
    #[must_use]
    pub fn active_count(&self) -> usize {
        self.tracks
            .iter()
            .filter(|t| t.lifecycle().is_active())
            .count()
    }

    /// Returns the number of confirmed tracks.
    #[must_use]
    pub fn confirmed_count(&self) -> usize {
        self.tracks
            .iter()
            .filter(|t| t.lifecycle().is_confirmed())
            .count()
    }

    /// Gets a track by ID.
    #[must_use]
    pub fn get(&self, id: TrackId) -> Option<&TrackingState> {
        self.tracks.iter().find(|t| t.track_id() == id)
    }

    /// Gets a mutable track by ID.
    pub fn get_mut(&mut self, id: TrackId) -> Option<&mut TrackingState> {
        self.tracks.iter_mut().find(|t| t.track_id() == id)
    }

    /// Returns all active tracks.
    #[must_use]
    pub fn active_tracks(&self) -> Vec<&TrackingState> {
        self.tracks
            .iter()
            .filter(|t| t.lifecycle().is_active())
            .collect()
    }

    /// Returns all confirmed tracks (for output).
    #[must_use]
    pub fn confirmed_tracks(&self) -> Vec<&TrackingState> {
        self.tracks
            .iter()
            .filter(|t| t.lifecycle().is_confirmed())
            .collect()
    }

    /// Updates unmatched tracks and removes deleted ones.
    pub fn update_unmatched_tracks(&mut self, matched_ids: &[TrackId]) {
        for track in &mut self.tracks {
            if !matched_ids.contains(&track.track_id()) && track.lifecycle().is_active() {
                track.update_unmatched(self.max_age);
            }
        }
    }

    /// Removes all deleted tracks from the registry.
    pub fn prune_deleted(&mut self) {
        self.tracks.retain(|t| !t.should_delete());
    }

    /// Clears all tracks and resets the ID counter.
    pub fn clear(&mut self) {
        self.tracks.clear();
        self.next_id = 1;
    }

    /// Returns the maximum age parameter.
    #[must_use]
    pub const fn max_age(&self) -> u32 {
        self.max_age
    }

    /// Returns the minimum hits parameter.
    #[must_use]
    pub const fn min_hits(&self) -> u32 {
        self.min_hits
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_lossless,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;

    fn sample_bbox() -> BoundingBox {
        BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0)
    }

    #[test]
    fn track_id_new() {
        let id = TrackId::new(42);
        assert_eq!(id.as_u64(), 42);
    }

    #[test]
    fn track_id_from() {
        let id: TrackId = 123u64.into();
        let back: u64 = id.into();
        assert_eq!(back, 123);
    }

    #[test]
    fn track_id_display() {
        let id = TrackId::new(7);
        assert_eq!(format!("{id}"), "Track(7)");
    }

    #[test]
    fn track_lifecycle_is_active() {
        assert!(TrackLifecycle::Tentative.is_active());
        assert!(TrackLifecycle::Confirmed.is_active());
        assert!(!TrackLifecycle::Lost.is_active());
        assert!(!TrackLifecycle::Deleted.is_active());
    }

    #[test]
    fn track_lifecycle_is_confirmed() {
        assert!(!TrackLifecycle::Tentative.is_confirmed());
        assert!(TrackLifecycle::Confirmed.is_confirmed());
        assert!(!TrackLifecycle::Lost.is_confirmed());
        assert!(!TrackLifecycle::Deleted.is_confirmed());
    }

    #[test]
    fn tracking_state_new() {
        let bbox = sample_bbox();
        let state = TrackingState::new(TrackId::new(1), bbox, 0);

        assert_eq!(state.track_id().as_u64(), 1);
        assert_eq!(state.lifecycle(), TrackLifecycle::Tentative);
        assert_eq!(state.created_frame(), 0);
        assert_eq!(state.total_matches(), 1);
    }

    #[test]
    fn tracking_state_predict() {
        let bbox = sample_bbox();
        let mut state = TrackingState::new(TrackId::new(1), bbox, 0);
        state.velocity = [0.1, 0.05, 0.0, 0.0];

        let predicted = state.predict();
        assert!((predicted.x0 - 0.2).abs() < 1e-6);
        assert!((predicted.y0 - 0.25).abs() < 1e-6);
    }

    #[test]
    fn tracking_state_update_matched() {
        let bbox = sample_bbox();
        let mut state = TrackingState::new(TrackId::new(1), bbox, 0);

        // Update multiple times to confirm
        for frame in 1..=3 {
            let new_bbox = BoundingBox::new(
                0.01f32.mul_add(frame as f32, 0.1),
                0.2,
                0.01f32.mul_add(frame as f32, 0.3),
                0.4,
                0.9,
                0,
            );
            state.update_matched(&new_bbox, frame, 0.5);
        }

        assert_eq!(state.lifecycle(), TrackLifecycle::Confirmed);
        assert_eq!(state.total_matches(), 4);
    }

    #[test]
    fn tracking_state_update_unmatched() {
        let bbox = sample_bbox();
        let mut state = TrackingState::new(TrackId::new(1), bbox, 0);

        // Tentative track should be deleted immediately
        state.update_unmatched(30);
        assert_eq!(state.lifecycle(), TrackLifecycle::Deleted);
    }

    #[test]
    fn tracking_state_confirmed_goes_lost() {
        let bbox = sample_bbox();
        let mut state = TrackingState::new(TrackId::new(1), bbox, 0);

        // Confirm the track
        for frame in 1..=3 {
            state.update_matched(&bbox, frame, 0.5);
        }
        assert_eq!(state.lifecycle(), TrackLifecycle::Confirmed);

        // Now miss a frame
        state.update_unmatched(30);
        assert_eq!(state.lifecycle(), TrackLifecycle::Lost);
    }

    #[test]
    fn tracking_state_age() {
        let bbox = sample_bbox();
        let state = TrackingState::new(TrackId::new(1), bbox, 10);

        assert_eq!(state.age(15), 5);
        assert_eq!(state.age(10), 0);
    }

    #[test]
    fn track_registry_new() {
        let registry = TrackRegistry::new();
        assert_eq!(registry.len(), 0);
        assert!(registry.is_empty());
    }

    #[test]
    fn track_registry_create_track() {
        let mut registry = TrackRegistry::new();
        let bbox = sample_bbox();

        let id1 = registry.create_track(bbox, 0);
        let id2 = registry.create_track(bbox, 0);

        assert_eq!(id1.as_u64(), 1);
        assert_eq!(id2.as_u64(), 2);
        assert_eq!(registry.len(), 2);
    }

    #[test]
    fn track_registry_get() {
        let mut registry = TrackRegistry::new();
        let bbox = sample_bbox();

        let id = registry.create_track(bbox, 0);
        let track = registry.get(id);

        assert!(track.is_some());
        assert_eq!(track.map(super::TrackingState::track_id), Some(id));
    }

    #[test]
    fn track_registry_active_count() {
        let mut registry = TrackRegistry::new();
        let bbox = sample_bbox();

        registry.create_track(bbox, 0);
        registry.create_track(bbox, 0);

        assert_eq!(registry.active_count(), 2);
    }

    #[test]
    fn track_registry_prune_deleted() {
        let mut registry = TrackRegistry::new();
        let bbox = sample_bbox();

        let id = registry.create_track(bbox, 0);
        registry.update_unmatched_tracks(&[]); // This will delete tentative track

        assert_eq!(registry.len(), 1);
        assert!(
            registry
                .get(id)
                .is_some_and(super::TrackingState::should_delete)
        );

        registry.prune_deleted();
        assert_eq!(registry.len(), 0);
    }

    #[test]
    fn track_registry_clear() {
        let mut registry = TrackRegistry::new();
        let bbox = sample_bbox();

        registry.create_track(bbox, 0);
        registry.create_track(bbox, 0);
        registry.clear();

        assert_eq!(registry.len(), 0);
        assert!(registry.is_empty());
    }

    #[test]
    fn track_id_serialization() {
        let id = TrackId::new(42);
        let json = serde_json::to_string(&id);
        assert!(json.is_ok());

        let parsed: Result<TrackId, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or(TrackId::new(0)), id);
    }

    #[test]
    fn tracking_state_serialization() {
        let bbox = sample_bbox();
        let state = TrackingState::new(TrackId::new(1), bbox, 0);

        let json = serde_json::to_string(&state);
        assert!(json.is_ok());

        let parsed: Result<TrackingState, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }

    #[test]
    fn track_registry_serialization() {
        let mut registry = TrackRegistry::new();
        let bbox = sample_bbox();
        registry.create_track(bbox, 0);

        let json = serde_json::to_string(&registry);
        assert!(json.is_ok());

        let parsed: Result<TrackRegistry, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
