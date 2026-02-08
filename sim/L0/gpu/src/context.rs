//! GPU context management for simulation compute shaders.
//!
//! Provides lazy initialization of the GPU device and queue, with automatic
//! fallback detection when GPU is unavailable. Follows the `mesh-gpu`
//! singleton pattern.

use std::sync::OnceLock;

use tracing::{debug, info, warn};
use wgpu::{Device, DeviceDescriptor, Instance, Queue, RequestAdapterOptions};

/// Global GPU context for simulation, lazily initialized on first access.
static GPU_SIM_CONTEXT: OnceLock<Option<GpuSimContext>> = OnceLock::new();

/// GPU device context for simulation compute shaders.
///
/// Initialized lazily on first access via [`GpuSimContext::get()`].
/// Returns `None` if no compatible GPU is available.
///
/// Separate from `mesh_gpu::GpuContext` because simulation may request
/// different device features/limits in the future (e.g., f64 support
/// for validation mode).
pub struct GpuSimContext {
    /// The wgpu device for creating resources and pipelines.
    pub(crate) device: Device,
    /// The command queue for submitting work.
    pub(crate) queue: Queue,
    /// Device limits for resource allocation.
    pub(crate) limits: wgpu::Limits,
    /// Adapter info (GPU name, vendor, backend) for logging/debugging.
    adapter_info: wgpu::AdapterInfo,
}

impl GpuSimContext {
    /// Returns a reference to the singleton context, or `None` if no
    /// compatible GPU is available. Initializes on first call.
    #[must_use]
    pub fn get() -> Option<&'static Self> {
        GPU_SIM_CONTEXT
            .get_or_init(|| match pollster::block_on(Self::try_init()) {
                Ok(ctx) => {
                    info!(
                        adapter = %ctx.adapter_info.name,
                        backend = ?ctx.adapter_info.backend,
                        "GPU simulation context initialized"
                    );
                    Some(ctx)
                }
                Err(msg) => {
                    warn!("GPU simulation initialization failed: {}", msg);
                    None
                }
            })
            .as_ref()
    }

    /// Convenience: `Self::get().is_some()`.
    #[must_use]
    pub fn is_available() -> bool {
        Self::get().is_some()
    }

    /// Adapter information (GPU name, vendor, device type, backend).
    /// Useful for logging which GPU is being used in RL training runs.
    #[must_use]
    pub const fn adapter_info(&self) -> &wgpu::AdapterInfo {
        &self.adapter_info
    }

    /// Try to initialize a new GPU context.
    async fn try_init() -> Result<Self, String> {
        debug!("Initializing GPU simulation context");

        let instance = Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = instance
            .request_adapter(&RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: None,
            })
            .await
            .ok_or_else(|| "no compatible GPU adapter found".to_string())?;

        let adapter_info = adapter.get_info();
        debug!(
            name = %adapter_info.name,
            vendor = adapter_info.vendor,
            device_type = ?adapter_info.device_type,
            backend = ?adapter_info.backend,
            "GPU adapter found for simulation"
        );

        let (device, queue) = adapter
            .request_device(
                &DeviceDescriptor {
                    label: Some("sim-gpu"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    ..Default::default()
                },
                None,
            )
            .await
            .map_err(|e| format!("device request failed: {e}"))?;

        let limits = device.limits();

        Ok(Self {
            device,
            queue,
            limits,
            adapter_info,
        })
    }
}

impl std::fmt::Debug for GpuSimContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GpuSimContext")
            .field("adapter", &self.adapter_info.name)
            .field(
                "max_storage_buffer",
                &self.limits.max_storage_buffer_binding_size,
            )
            .finish_non_exhaustive()
    }
}
