//! GPS sensor types.
//!
//! Provides types for global positioning data.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::Timestamp;

/// GPS accuracy estimates.
///
/// Describes the uncertainty in GPS position measurements.
///
/// # Example
///
/// ```
/// use sensor_types::GpsAccuracy;
///
/// let accuracy = GpsAccuracy {
///     horizontal: 2.5,
///     vertical: Some(5.0),
///     speed: Some(0.1),
///     heading: Some(2.0),
/// };
///
/// assert!(accuracy.is_high_precision(5.0));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GpsAccuracy {
    /// Horizontal accuracy (CEP) in meters.
    pub horizontal: f64,

    /// Vertical accuracy in meters (optional).
    pub vertical: Option<f64>,

    /// Speed accuracy in m/s (optional).
    pub speed: Option<f64>,

    /// Heading accuracy in degrees (optional).
    pub heading: Option<f64>,
}

impl GpsAccuracy {
    /// Creates a new GPS accuracy estimate.
    #[must_use]
    pub const fn new(horizontal: f64) -> Self {
        Self {
            horizontal,
            vertical: None,
            speed: None,
            heading: None,
        }
    }

    /// Creates a GPS accuracy with horizontal and vertical estimates.
    #[must_use]
    pub const fn horizontal_vertical(horizontal: f64, vertical: f64) -> Self {
        Self {
            horizontal,
            vertical: Some(vertical),
            speed: None,
            heading: None,
        }
    }

    /// Checks if the horizontal accuracy is better than a threshold.
    #[must_use]
    pub fn is_high_precision(&self, threshold_meters: f64) -> bool {
        self.horizontal < threshold_meters
    }

    /// Returns the 3D accuracy (Euclidean norm of horizontal and vertical).
    ///
    /// Returns `None` if vertical accuracy isn't available.
    #[must_use]
    pub fn accuracy_3d(&self) -> Option<f64> {
        self.vertical.map(|v| self.horizontal.hypot(v))
    }
}

impl Default for GpsAccuracy {
    fn default() -> Self {
        Self::new(10.0) // Typical GPS accuracy
    }
}

/// GPS fix quality/type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum GpsFixType {
    /// No fix available.
    #[default]
    NoFix,
    /// 2D fix (latitude/longitude only).
    Fix2d,
    /// 3D fix (latitude/longitude/altitude).
    Fix3d,
    /// Differential GPS (DGPS) fix.
    Dgps,
    /// Real-time kinematic (RTK) float solution.
    RtkFloat,
    /// Real-time kinematic (RTK) fixed solution (highest accuracy).
    RtkFixed,
}

impl GpsFixType {
    /// Returns `true` if there is any valid fix.
    #[must_use]
    pub const fn has_fix(self) -> bool {
        !matches!(self, Self::NoFix)
    }

    /// Returns `true` if the fix includes altitude.
    #[must_use]
    pub const fn has_altitude(self) -> bool {
        matches!(
            self,
            Self::Fix3d | Self::Dgps | Self::RtkFloat | Self::RtkFixed
        )
    }

    /// Returns `true` if this is an RTK fix (highest accuracy).
    #[must_use]
    pub const fn is_rtk(self) -> bool {
        matches!(self, Self::RtkFloat | Self::RtkFixed)
    }
}

/// A GPS position reading.
///
/// Contains latitude, longitude, and optional altitude with accuracy estimates.
///
/// # Coordinate System
///
/// - Latitude: degrees, positive = North, negative = South
/// - Longitude: degrees, positive = East, negative = West
/// - Altitude: meters above WGS84 ellipsoid
///
/// # Example
///
/// ```
/// use sensor_types::{GpsReading, GpsAccuracy, GpsFixType, Timestamp};
///
/// let reading = GpsReading {
///     timestamp: Timestamp::from_secs_f64(1.0),
///     latitude: 37.7749,      // San Francisco
///     longitude: -122.4194,
///     altitude: Some(10.0),
///     accuracy: Some(GpsAccuracy::new(2.5)),
///     fix_type: GpsFixType::Fix3d,
///     speed: Some(1.5),
///     heading: Some(45.0),
///     satellites: Some(12),
/// };
///
/// assert!(reading.has_fix());
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct GpsReading {
    /// Timestamp of the reading.
    pub timestamp: Timestamp,

    /// Latitude in degrees (-90 to 90).
    pub latitude: f64,

    /// Longitude in degrees (-180 to 180).
    pub longitude: f64,

    /// Altitude in meters above WGS84 ellipsoid (optional).
    pub altitude: Option<f64>,

    /// Accuracy estimates (optional).
    pub accuracy: Option<GpsAccuracy>,

    /// GPS fix type.
    pub fix_type: GpsFixType,

    /// Ground speed in m/s (optional).
    pub speed: Option<f64>,

    /// Heading/course in degrees (0-360, 0=North, optional).
    pub heading: Option<f64>,

    /// Number of satellites used in fix (optional).
    pub satellites: Option<u8>,
}

impl GpsReading {
    /// Creates a minimal GPS reading with just coordinates.
    #[must_use]
    pub const fn new(timestamp: Timestamp, latitude: f64, longitude: f64) -> Self {
        Self {
            timestamp,
            latitude,
            longitude,
            altitude: None,
            accuracy: None,
            fix_type: GpsFixType::Fix2d,
            speed: None,
            heading: None,
            satellites: None,
        }
    }

    /// Creates a GPS reading with altitude.
    #[must_use]
    pub const fn with_altitude(
        timestamp: Timestamp,
        latitude: f64,
        longitude: f64,
        altitude: f64,
    ) -> Self {
        Self {
            timestamp,
            latitude,
            longitude,
            altitude: Some(altitude),
            accuracy: None,
            fix_type: GpsFixType::Fix3d,
            speed: None,
            heading: None,
            satellites: None,
        }
    }

    /// Returns `true` if there is a valid GPS fix.
    #[must_use]
    pub const fn has_fix(&self) -> bool {
        self.fix_type.has_fix()
    }

    /// Returns `true` if altitude is available.
    #[must_use]
    pub const fn has_altitude(&self) -> bool {
        self.altitude.is_some()
    }

    /// Returns the position as `[latitude, longitude]`.
    #[must_use]
    pub const fn position_2d(&self) -> [f64; 2] {
        [self.latitude, self.longitude]
    }

    /// Returns the position as `[latitude, longitude, altitude]`.
    ///
    /// Returns `None` if altitude isn't available.
    #[must_use]
    pub fn position_3d(&self) -> Option<[f64; 3]> {
        self.altitude
            .map(|alt| [self.latitude, self.longitude, alt])
    }

    /// Computes the horizontal accuracy in meters.
    ///
    /// Returns `None` if accuracy isn't available.
    #[must_use]
    pub fn horizontal_accuracy(&self) -> Option<f64> {
        self.accuracy.as_ref().map(|a| a.horizontal)
    }

    /// Validates the coordinate ranges.
    ///
    /// Returns `true` if latitude is in [-90, 90] and longitude is in [-180, 180].
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.latitude >= -90.0
            && self.latitude <= 90.0
            && self.longitude >= -180.0
            && self.longitude <= 180.0
            && !self.latitude.is_nan()
            && !self.longitude.is_nan()
    }

    /// Computes the approximate distance to another GPS position.
    ///
    /// Uses the haversine formula for accuracy on a spherical Earth.
    /// Returns distance in meters.
    #[must_use]
    pub fn distance_to(&self, other: &Self) -> f64 {
        haversine_distance(
            self.latitude,
            self.longitude,
            other.latitude,
            other.longitude,
        )
    }

    /// Computes the initial bearing to another GPS position.
    ///
    /// Returns bearing in degrees (0-360, 0=North).
    #[must_use]
    pub fn bearing_to(&self, other: &Self) -> f64 {
        let lat1 = self.latitude.to_radians();
        let lat2 = other.latitude.to_radians();
        let dlon = (other.longitude - self.longitude).to_radians();

        let y = dlon.sin() * lat2.cos();
        let x = lat1
            .cos()
            .mul_add(lat2.sin(), -(lat1.sin() * lat2.cos() * dlon.cos()));

        let bearing = y.atan2(x).to_degrees();
        (bearing + 360.0) % 360.0
    }
}

impl Default for GpsReading {
    fn default() -> Self {
        Self {
            timestamp: Timestamp::zero(),
            latitude: 0.0,
            longitude: 0.0,
            altitude: None,
            accuracy: None,
            fix_type: GpsFixType::NoFix,
            speed: None,
            heading: None,
            satellites: None,
        }
    }
}

/// Computes the haversine distance between two coordinates.
///
/// Returns distance in meters.
#[must_use]
fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const EARTH_RADIUS_M: f64 = 6_371_000.0;

    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();

    let sin_dlat_half = (dlat / 2.0).sin();
    let sin_dlon_half = (dlon / 2.0).sin();
    let a = sin_dlat_half.mul_add(
        sin_dlat_half,
        lat1_rad.cos() * lat2_rad.cos() * sin_dlon_half * sin_dlon_half,
    );
    let c = 2.0 * a.sqrt().asin();

    EARTH_RADIUS_M * c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_accuracy_new() {
        let acc = GpsAccuracy::new(2.5);
        assert!((acc.horizontal - 2.5).abs() < 1e-10);
        assert!(acc.vertical.is_none());
    }

    #[test]
    fn gps_accuracy_3d() {
        let acc = GpsAccuracy::horizontal_vertical(3.0, 4.0);
        assert!(acc.accuracy_3d().is_some());
        assert!((acc.accuracy_3d().unwrap_or(0.0) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn gps_fix_type() {
        assert!(!GpsFixType::NoFix.has_fix());
        assert!(GpsFixType::Fix2d.has_fix());
        assert!(GpsFixType::Fix3d.has_altitude());
        assert!(GpsFixType::RtkFixed.is_rtk());
    }

    #[test]
    fn gps_reading_new() {
        let reading = GpsReading::new(Timestamp::zero(), 37.7749, -122.4194);
        assert!(reading.has_fix());
        assert!(!reading.has_altitude());
        assert!(reading.is_valid());
    }

    #[test]
    fn gps_reading_with_altitude() {
        let reading = GpsReading::with_altitude(Timestamp::zero(), 37.7749, -122.4194, 10.0);
        assert!(reading.has_altitude());
        let pos = reading.position_3d();
        assert!(pos.is_some());
    }

    #[test]
    fn gps_reading_validity() {
        let valid = GpsReading::new(Timestamp::zero(), 37.7749, -122.4194);
        assert!(valid.is_valid());

        let invalid_lat = GpsReading::new(Timestamp::zero(), 91.0, 0.0);
        assert!(!invalid_lat.is_valid());

        let invalid_lon = GpsReading::new(Timestamp::zero(), 0.0, 181.0);
        assert!(!invalid_lon.is_valid());

        let nan = GpsReading::new(Timestamp::zero(), f64::NAN, 0.0);
        assert!(!nan.is_valid());
    }

    #[test]
    fn gps_distance() {
        // San Francisco
        let sf = GpsReading::new(Timestamp::zero(), 37.7749, -122.4194);
        // Los Angeles
        let la = GpsReading::new(Timestamp::zero(), 34.0522, -118.2437);

        let distance = sf.distance_to(&la);
        // Approximately 560 km
        assert!(distance > 550_000.0 && distance < 580_000.0);
    }

    #[test]
    fn gps_bearing() {
        let a = GpsReading::new(Timestamp::zero(), 0.0, 0.0);
        let north = GpsReading::new(Timestamp::zero(), 1.0, 0.0);

        let bearing = a.bearing_to(&north);
        assert!(bearing.abs() < 1.0 || (bearing - 360.0).abs() < 1.0); // Should be ~0 (North)
    }

    #[test]
    fn gps_same_location() {
        let a = GpsReading::new(Timestamp::zero(), 37.7749, -122.4194);
        let b = GpsReading::new(Timestamp::zero(), 37.7749, -122.4194);

        let distance = a.distance_to(&b);
        assert!(distance < 0.001); // Should be essentially 0
    }

    #[cfg(feature = "serde")]
    #[test]
    fn gps_serialization() {
        let reading = GpsReading::with_altitude(Timestamp::zero(), 37.7749, -122.4194, 10.0);
        let json = serde_json::to_string(&reading).ok();
        assert!(json.is_some());

        let parsed: Result<GpsReading, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
