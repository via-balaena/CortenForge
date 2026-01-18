//! GPU context management for mesh processing.
//!
//! This module provides lazy initialization of the GPU device and queue,
//! with automatic fallback detection when GPU is unavailable.
//!
//! # Example
//!
//! ```no_run
//! use mesh_gpu::context::GpuContext;
//!
//! if let Some(ctx) = GpuContext::get() {
//!     println!("GPU available: {}", ctx.adapter_info.name);
//! } else {
//!     println!("No GPU available");
//! }
//! ```

use std::sync::OnceLock;

use tracing::{debug, info, warn};
use wgpu::{Device, DeviceDescriptor, Instance, Queue, RequestAdapterOptions};

use crate::error::{GpuError, GpuResult};

/// Global GPU context, lazily initialized on first access.
static GPU_CONTEXT: OnceLock<Option<GpuContext>> = OnceLock::new();

/// GPU device preference for adapter selection.
///
/// This controls which GPU is selected when multiple are available.
///
/// # Example
///
/// ```
/// use mesh_gpu::context::GpuDevicePreference;
///
/// // Default selects the best available device
/// assert_eq!(GpuDevicePreference::default(), GpuDevicePreference::Auto);
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum GpuDevicePreference {
    /// Automatically select the best available device.
    ///
    /// This typically selects a discrete GPU if available, falling back
    /// to integrated graphics otherwise.
    #[default]
    Auto,

    /// Prefer a high-performance discrete GPU.
    ///
    /// This is best for compute-intensive workloads where power consumption
    /// is not a concern.
    HighPerformance,

    /// Prefer a low-power integrated GPU.
    ///
    /// This is useful for laptops or when power efficiency is important.
    LowPower,
}

/// Information about the GPU adapter.
///
/// This provides details about the selected GPU hardware and driver.
///
/// # Example
///
/// ```no_run
/// use mesh_gpu::context::GpuContext;
///
/// if let Some(ctx) = GpuContext::get() {
///     println!("GPU: {} ({})", ctx.adapter_info.name, ctx.adapter_info.backend);
/// }
/// ```
#[derive(Debug, Clone)]
pub struct GpuAdapterInfo {
    /// Device name (e.g., "NVIDIA RTX 3080").
    pub name: String,

    /// Vendor identifier.
    pub vendor: String,

    /// Device type (e.g., Discrete, Integrated).
    pub device_type: String,

    /// Backend API (e.g., Vulkan, Metal, Dx12).
    pub backend: String,
}

impl From<wgpu::AdapterInfo> for GpuAdapterInfo {
    fn from(info: wgpu::AdapterInfo) -> Self {
        Self {
            name: info.name,
            vendor: format!("{}", info.vendor),
            device_type: format!("{:?}", info.device_type),
            backend: format!("{:?}", info.backend),
        }
    }
}

/// GPU context containing device, queue, and adapter information.
///
/// This is a lazy-initialized singleton that provides access to GPU resources.
/// Use [`GpuContext::get()`] to access the global context, or [`GpuContext::try_get()`]
/// for error handling.
///
/// # Thread Safety
///
/// The context is initialized once and is safe to access from multiple threads.
/// The underlying WGPU device and queue are thread-safe.
///
/// # Example
///
/// ```no_run
/// use mesh_gpu::context::GpuContext;
///
/// // Check availability
/// if GpuContext::is_available() {
///     let ctx = GpuContext::get().expect("GPU available");
///     println!("Max buffer size: {} bytes", ctx.max_buffer_size());
/// }
/// ```
pub struct GpuContext {
    /// The WGPU device for creating resources and pipelines.
    pub device: Device,

    /// The command queue for submitting work.
    pub queue: Queue,

    /// Information about the GPU adapter.
    pub adapter_info: GpuAdapterInfo,

    /// Device limits for resource allocation.
    pub limits: wgpu::Limits,
}

impl GpuContext {
    /// Get or initialize the global GPU context.
    ///
    /// Returns `Some(&GpuContext)` if a GPU is available, `None` otherwise.
    /// The context is lazily initialized on first call.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_gpu::context::GpuContext;
    ///
    /// if let Some(ctx) = GpuContext::get() {
    ///     println!("GPU: {}", ctx.adapter_info.name);
    /// } else {
    ///     println!("No GPU available");
    /// }
    /// ```
    #[must_use]
    pub fn get() -> Option<&'static Self> {
        GPU_CONTEXT
            .get_or_init(
                || match pollster::block_on(Self::try_init(GpuDevicePreference::Auto)) {
                    Ok(ctx) => {
                        info!(
                            adapter = %ctx.adapter_info.name,
                            backend = %ctx.adapter_info.backend,
                            "GPU context initialized"
                        );
                        Some(ctx)
                    }
                    Err(e) => {
                        warn!("GPU initialization failed: {}", e);
                        None
                    }
                },
            )
            .as_ref()
    }

    /// Try to get the global GPU context, returning an error if unavailable.
    ///
    /// # Errors
    ///
    /// Returns [`GpuError::NotAvailable`] if no GPU is available.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_gpu::context::GpuContext;
    ///
    /// match GpuContext::try_get() {
    ///     Ok(ctx) => println!("GPU: {}", ctx.adapter_info.name),
    ///     Err(e) => println!("GPU error: {}", e),
    /// }
    /// ```
    pub fn try_get() -> GpuResult<&'static Self> {
        Self::get().ok_or(GpuError::NotAvailable)
    }

    /// Check if a GPU is available without initializing the context.
    ///
    /// Note: This will initialize the context on first call to determine availability.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_gpu::context::GpuContext;
    ///
    /// if GpuContext::is_available() {
    ///     println!("GPU computation is available");
    /// }
    /// ```
    #[must_use]
    pub fn is_available() -> bool {
        Self::get().is_some()
    }

    /// Try to initialize a new GPU context with the specified device preference.
    async fn try_init(preference: GpuDevicePreference) -> GpuResult<Self> {
        debug!("Initializing GPU context with preference: {:?}", preference);

        // Create WGPU instance with all available backends
        let instance = Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        // Request adapter based on preference
        let power_preference = match preference {
            GpuDevicePreference::Auto | GpuDevicePreference::HighPerformance => {
                wgpu::PowerPreference::HighPerformance
            }
            GpuDevicePreference::LowPower => wgpu::PowerPreference::LowPower,
        };

        let adapter = instance
            .request_adapter(&RequestAdapterOptions {
                power_preference,
                force_fallback_adapter: false,
                compatible_surface: None,
            })
            .await
            .ok_or(GpuError::NotAvailable)?;

        let adapter_info = adapter.get_info();
        debug!(
            name = %adapter_info.name,
            vendor = adapter_info.vendor,
            device_type = ?adapter_info.device_type,
            backend = ?adapter_info.backend,
            "GPU adapter found"
        );

        // Request device with default limits
        let (device, queue) = adapter
            .request_device(
                &DeviceDescriptor {
                    label: Some("mesh-gpu"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    ..Default::default()
                },
                None,
            )
            .await
            .map_err(|e| GpuError::Execution(format!("device request failed: {e}")))?;

        let limits = device.limits();

        Ok(Self {
            device,
            queue,
            adapter_info: adapter_info.into(),
            limits,
        })
    }

    /// Get the maximum buffer size supported by this device.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use mesh_gpu::context::GpuContext;
    ///
    /// if let Some(ctx) = GpuContext::get() {
    ///     println!("Max buffer: {} bytes", ctx.max_buffer_size());
    /// }
    /// ```
    #[must_use]
    pub const fn max_buffer_size(&self) -> u64 {
        self.limits.max_buffer_size
    }

    /// Get the maximum storage buffer binding size.
    ///
    /// This is the maximum size for a single storage buffer used in compute shaders.
    #[must_use]
    pub const fn max_storage_buffer_size(&self) -> u32 {
        self.limits.max_storage_buffer_binding_size
    }

    /// Get the maximum compute workgroup size in each dimension.
    ///
    /// Returns `[x, y, z]` limits for workgroup dimensions.
    #[must_use]
    pub const fn max_workgroup_size(&self) -> [u32; 3] {
        [
            self.limits.max_compute_workgroup_size_x,
            self.limits.max_compute_workgroup_size_y,
            self.limits.max_compute_workgroup_size_z,
        ]
    }

    /// Get the maximum number of compute invocations per workgroup.
    #[must_use]
    pub const fn max_invocations_per_workgroup(&self) -> u32 {
        self.limits.max_compute_invocations_per_workgroup
    }

    /// Estimate available GPU memory (conservative estimate).
    ///
    /// Note: WGPU doesn't expose exact memory info, so this returns a
    /// conservative estimate based on buffer limits.
    #[must_use]
    pub const fn estimate_available_memory(&self) -> u64 {
        // Use max buffer size as a proxy for available memory
        // In practice, we can often allocate more via multiple buffers
        self.limits.max_buffer_size
    }
}

impl std::fmt::Debug for GpuContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuContext")
            .field("adapter_info", &self.adapter_info)
            .field("max_buffer_size", &self.limits.max_buffer_size)
            .finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpu_availability_check() {
        // This test just checks that is_available() doesn't panic
        let _available = GpuContext::is_available();
    }

    #[test]
    fn test_gpu_context_get() {
        // This test checks that get() returns consistent results
        let first = GpuContext::get();
        let second = GpuContext::get();

        // Both calls should return the same result
        assert_eq!(first.is_some(), second.is_some());

        if let Some(ctx) = first {
            assert!(!ctx.adapter_info.name.is_empty());
        }
    }

    #[test]
    fn test_gpu_device_preference_default() {
        assert_eq!(GpuDevicePreference::default(), GpuDevicePreference::Auto);
    }

    #[test]
    fn test_gpu_device_preference_variants() {
        // Ensure all variants can be created and compared
        let auto = GpuDevicePreference::Auto;
        let high = GpuDevicePreference::HighPerformance;
        let low = GpuDevicePreference::LowPower;

        assert_ne!(auto, high);
        assert_ne!(high, low);
        assert_ne!(auto, low);
    }

    #[test]
    fn test_adapter_info_debug() {
        let info = GpuAdapterInfo {
            name: "Test GPU".to_string(),
            vendor: "12345".to_string(),
            device_type: "DiscreteGpu".to_string(),
            backend: "Vulkan".to_string(),
        };

        let debug_str = format!("{info:?}");
        assert!(debug_str.contains("Test GPU"));
        assert!(debug_str.contains("Vulkan"));
    }

    #[test]
    fn test_context_limits_if_available() {
        if let Some(ctx) = GpuContext::get() {
            // Verify limits are reasonable
            assert!(ctx.max_buffer_size() > 0);
            assert!(ctx.max_storage_buffer_size() > 0);

            let workgroup = ctx.max_workgroup_size();
            assert!(workgroup[0] > 0);
            assert!(workgroup[1] > 0);
            assert!(workgroup[2] > 0);

            assert!(ctx.max_invocations_per_workgroup() > 0);
            assert!(ctx.estimate_available_memory() > 0);
        }
    }
}
