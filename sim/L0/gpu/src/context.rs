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
        let adapter = request_adapter().await?;

        let adapter_info = adapter.get_info();
        let adapter_limits = adapter.limits();

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("sim-gpu-physics"),
                required_features: wgpu::Features::empty(),
                // Raise storage buffer limit for physics pipeline (FK uses
                // ~12 storage buffers per shader stage). Metal supports 31,
                // Vulkan/NVIDIA supports many more. WebGPU default is 8.
                //
                // Also request the adapter's full buffer-size limits (not wgpu's
                // 256 MB default): batched state buffers scale with n_env, so the
                // 256 MB cap otherwise pins the batch at ~16 environments. Apple
                // and discrete GPUs support multi-GB buffers; take what's offered.
                required_limits: wgpu::Limits {
                    max_storage_buffers_per_shader_stage: 16,
                    max_buffer_size: adapter_limits.max_buffer_size,
                    max_storage_buffer_binding_size: adapter_limits.max_storage_buffer_binding_size,
                    ..wgpu::Limits::default()
                },
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

/// The adapter every context is built on: high-performance, Metal or Vulkan.
async fn request_adapter() -> Result<wgpu::Adapter, GpuError> {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends: wgpu::Backends::METAL | wgpu::Backends::VULKAN,
        ..Default::default()
    });
    instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        })
        .await
        .map_err(|_| GpuError::NoAdapter)
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
    /// ★ The context is on the platform's backend (Metal on macOS, Vulkan
    /// elsewhere, lavapipe in CI) and was granted the adapter's own buffer
    /// limits, which the soft executor's fine grid needs above wgpu's default.
    #[test]
    fn a_context_takes_the_platform_backend_and_the_adapters_buffer_limits() {
        let Some(ctx) = crate::test_support::gpu_context_or_skip("adapter probe") else {
            return;
        };
        let expected = if cfg!(target_os = "macos") {
            wgpu::Backend::Metal
        } else {
            wgpu::Backend::Vulkan
        };
        assert_eq!(
            ctx.adapter_info.backend, expected,
            "{}",
            ctx.adapter_info.name
        );

        let adapter = pollster::block_on(super::request_adapter()).expect("an adapter, as above");
        let (granted, offered) = (ctx.device.limits(), adapter.limits());
        assert_eq!(granted.max_buffer_size, offered.max_buffer_size);
        assert_eq!(
            granted.max_storage_buffer_binding_size,
            offered.max_storage_buffer_binding_size
        );
    }
}
