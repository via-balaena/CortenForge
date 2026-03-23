//! GPU device and queue initialization.
//!
//! Creates its own wgpu Device separate from any rendering device
//! (e.g. Bevy) to avoid contention between physics compute and
//! rendering. Works in headless (no-window) contexts.

/// GPU compute context — owns a wgpu Device and Queue.
pub struct GpuContext {
    /// The GPU device for creating buffers, pipelines, and dispatching work.
    pub device: wgpu::Device,
    /// The command queue for submitting GPU work.
    pub queue: wgpu::Queue,
    /// Information about the selected GPU adapter.
    pub adapter_info: wgpu::AdapterInfo,
}

impl GpuContext {
    /// Create a GPU context, requesting a high-performance compute-capable adapter.
    ///
    /// Callers should fall back to CPU collision detection when this fails.
    ///
    /// # Errors
    ///
    /// Returns [`GpuError::NoAdapter`] if no suitable GPU is found, or
    /// [`GpuError::DeviceRequest`] if the device cannot be created.
    pub fn new() -> Result<Self, GpuError> {
        pollster::block_on(Self::new_async())
    }

    async fn new_async() -> Result<Self, GpuError> {
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::METAL | wgpu::Backends::VULKAN,
            ..Default::default()
        });

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .map_err(|_| GpuError::NoAdapter)?;

        let adapter_info = adapter.get_info();

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("sim-gpu-collision"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
                trace: wgpu::Trace::Off,
                experimental_features: wgpu::ExperimentalFeatures::default(),
            })
            .await
            .map_err(|e: wgpu::RequestDeviceError| GpuError::DeviceRequest(e.to_string()))?;

        Ok(Self {
            device,
            queue,
            adapter_info,
        })
    }
}

/// Errors from GPU context creation.
#[derive(Debug)]
pub enum GpuError {
    /// No suitable GPU adapter found on this system.
    NoAdapter,
    /// GPU device request failed.
    DeviceRequest(String),
}

impl std::fmt::Display for GpuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoAdapter => write!(f, "no suitable GPU adapter found"),
            Self::DeviceRequest(msg) => write!(f, "GPU device request failed: {msg}"),
        }
    }
}

impl std::error::Error for GpuError {}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn gpu_context_creates_on_metal() {
        let ctx = GpuContext::new().expect("GPU context should create on Metal");
        eprintln!("  GPU adapter: {}", ctx.adapter_info.name);
        eprintln!("  Backend: {:?}", ctx.adapter_info.backend);
        assert!(!ctx.adapter_info.name.is_empty());
    }
}
