//! The loader's stack, and the depths it is sized for.
//!
//! Parsing and building recurse once per level of the body and frame tree.
//! Parsing 496 nested bodies (the deepest MuJoCo 3.5.0's XML reader loads)
//! took 20.7 MiB of stack at opt-level 0 and 3.3 MiB in the repository's dev
//! profile (research A4 §1.4, macOS arm64), more than a test thread's 2 MiB.
//! So every public entry point that recurses runs its work on a thread of its
//! own with [`LOADER_STACK_BYTES`], and the depths that reach it are bounded:
//! the XML by MuJoCo's reader's rule (`parser`), a tree built in code or by a
//! composite by [`MAX_BODY_TREE_DEPTH`].

/// The loader thread's stack: the deepest measured case, 20.7 MiB, three times
/// over. The reservation is virtual memory, committed only as it is touched.
pub const LOADER_STACK_BYTES: usize = 64 << 20;

/// The deepest body and frame tree the loader takes that did not come from
/// XML: built in code, or by a composite. MuJoCo states no limit (3.5.0 loads
/// a cable of 1000 bodies and fails one of 3000).
pub const MAX_BODY_TREE_DEPTH: usize = 4096;

/// `f`, run on a thread with [`LOADER_STACK_BYTES`] of stack; a panic in it is
/// raised again here. Where that thread cannot start, and on a target without
/// threads, `f` runs on the caller's.
pub fn on_large_stack<T: Send>(f: impl FnOnce() -> T + Send) -> T {
    #[cfg(target_family = "wasm")]
    {
        f()
    }
    #[cfg(not(target_family = "wasm"))]
    {
        let mut slot = Some(f);
        let ran = std::thread::scope(|scope| {
            let slot = &mut slot;
            let spawned = std::thread::Builder::new()
                .name("sim-mjcf-loader".into())
                .stack_size(LOADER_STACK_BYTES)
                .spawn_scoped(scope, move || slot.take().map(|f| f()));
            match spawned {
                Ok(handle) => match handle.join() {
                    Ok(value) => value,
                    Err(panic) => std::panic::resume_unwind(panic),
                },
                Err(_) => None,
            }
        });
        match (ran, slot) {
            (Some(value), _) => value,
            // The thread could not start, so `f` never left the slot.
            (None, Some(f)) => f(),
            (None, None) => unreachable!("the loader thread ran f and returned nothing"),
        }
    }
}
