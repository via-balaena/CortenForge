//! The loader's stack, and the depths it is sized for.
//!
//! Parsing and building recurse once per level of the body and frame tree.
//! Parsing 496 nested bodies (the deepest MuJoCo 3.5.0's XML reader loads)
//! took 20,735 KB of stack at opt-level 0 and 3,327 KB in the repository's dev
//! profile (research A4 §1.4, macOS arm64), more than a test thread's 2 MiB.
//! So the entry points that parse, expand includes, validate or build
//! (`parse_mjcf_str`, `load_model`, `load_model_from_file`, `model_from_mjcf`,
//! `validate`), and the `.mjb` readers and writers (`mjb`), run their work on
//! a thread of their own with [`LOADER_STACK_BYTES`], and the depths that
//! reach it are bounded: the XML by MuJoCo's reader's rule (`parser`), a tree
//! built in code, by a composite or decoded from a `.mjb` by
//! [`MAX_BODY_TREE_DEPTH`].
//!
//! The loader thread takes the caller's tracing dispatcher and current span,
//! so what the loader logs reaches the caller's subscriber, within the
//! caller's span. A panic in it is raised again on the caller's thread; the
//! panic hook reports it from the thread `sim-mjcf-loader`.

/// The loader thread's stack: about three times the deepest measured case,
/// 20,735 KB. The reservation is virtual memory, committed only as it is
/// touched.
pub const LOADER_STACK_BYTES: usize = 64 << 20;

/// The deepest body and frame tree the loader takes that did not come from
/// XML: built in code, or by a composite. MuJoCo states no limit; 3.5.0 loads
/// a cable of count 1000 (999 bodies) and fails one of count 3000 with an
/// unknown exception, cause not isolated (research A4 §1.4).
pub const MAX_BODY_TREE_DEPTH: usize = 4096;

/// `f`, run on a thread with [`LOADER_STACK_BYTES`] of stack, in the caller's
/// tracing dispatcher and span; a panic in it is raised again here. Where that
/// thread cannot start, and on a target without threads, `f` runs on the
/// caller's stack, whose size then bounds the depths the loader can take below
/// its own limits.
pub fn on_large_stack<T: Send>(f: impl FnOnce() -> T + Send) -> T {
    on_stack_of(LOADER_STACK_BYTES, f)
}

/// [`on_large_stack`] with a stack of `bytes`.
fn on_stack_of<T: Send>(bytes: usize, f: impl FnOnce() -> T + Send) -> T {
    #[cfg(target_family = "wasm")]
    {
        let _ = bytes;
        f()
    }
    #[cfg(not(target_family = "wasm"))]
    {
        let dispatch = tracing::dispatcher::get_default(Clone::clone);
        let span = tracing::Span::current();
        let mut slot = Some(f);
        let ran = std::thread::scope(|scope| {
            let slot = &mut slot;
            let spawned = std::thread::Builder::new()
                .name("sim-mjcf-loader".into())
                .stack_size(bytes)
                .spawn_scoped(scope, move || {
                    tracing::dispatcher::with_default(&dispatch, || {
                        span.in_scope(|| slot.take().map(|f| f()))
                    })
                });
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

#[cfg(feature = "serde")]
thread_local! {
    /// The level of the body or frame this thread is decoding, the world's
    /// children at 1.
    static LEVEL: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    /// Whether a decode on this thread has met a level past
    /// [`MAX_BODY_TREE_DEPTH`].
    static TOO_DEEP: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// A body's or frame's bodies or frames, deserialized one level below their
/// holder: each level counted as [`crate::validation::check_tree_depth`]
/// counts it, and one past [`MAX_BODY_TREE_DEPTH`] refused before it is read,
/// so deserializing recurses no deeper than a tree the loader takes.
#[cfg(feature = "serde")]
pub fn nested<'de, D, T>(deserializer: D) -> Result<Vec<T>, D::Error>
where
    D: serde::Deserializer<'de>,
    T: serde::Deserialize<'de>,
{
    let levels: Vec<Level<T>> = serde::Deserialize::deserialize(deserializer)?;
    Ok(levels.into_iter().map(|level| level.0).collect())
}

/// One element of [`nested`]'s list, read one level deeper.
#[cfg(feature = "serde")]
struct Level<T>(T);

#[cfg(feature = "serde")]
impl<'de, T: serde::Deserialize<'de>> serde::Deserialize<'de> for Level<T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let level = LEVEL.with(|level| {
            level.set(level.get() + 1);
            level.get()
        });
        let _leave = LeaveLevel;
        if level > MAX_BODY_TREE_DEPTH {
            TOO_DEEP.with(|too_deep| too_deep.set(true));
            return Err(serde::de::Error::custom(format_args!(
                "nested deeper than {MAX_BODY_TREE_DEPTH}"
            )));
        }
        T::deserialize(deserializer).map(Level)
    }
}

/// Leaves the level [`Level`] entered, however its read ends.
#[cfg(feature = "serde")]
struct LeaveLevel;

#[cfg(feature = "serde")]
impl Drop for LeaveLevel {
    fn drop(&mut self) {
        LEVEL.with(|level| level.set(level.get().saturating_sub(1)));
    }
}

/// `decode`, and whether it met a level past [`MAX_BODY_TREE_DEPTH`] (its
/// error then says so).
#[cfg(feature = "mjb")]
pub fn decoding<T>(decode: impl FnOnce() -> T) -> (T, bool) {
    TOO_DEEP.with(|too_deep| too_deep.set(false));
    let decoded = decode();
    (decoded, TOO_DEEP.with(std::cell::Cell::get))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::{Arc, Mutex};
    use tracing::span::{Attributes, Id, Record};
    use tracing::{Event, Metadata, Subscriber};
    use tracing_core::span::Current;

    #[test]
    fn f_runs_on_the_loader_thread() {
        let name = on_large_stack(|| std::thread::current().name().map(str::to_owned));
        assert_eq!(name.as_deref(), Some("sim-mjcf-loader"));
    }

    #[test]
    fn a_panic_in_the_loader_reaches_the_caller_with_its_payload() {
        let caught = std::panic::catch_unwind(|| on_large_stack(|| panic!("from the loader")));
        let payload = caught.unwrap_err();
        assert_eq!(payload.downcast_ref::<&str>(), Some(&"from the loader"));
    }

    #[test]
    fn where_the_thread_cannot_start_f_runs_on_the_callers() {
        // A stack no system grants.
        let caller = std::thread::current().id();
        assert_eq!(on_stack_of(1 << 50, || std::thread::current().id()), caller);
    }

    /// Counts events, records which threads entered which spans, and keeps
    /// the span last entered and not exited as the current one, as a
    /// subscriber that tracks spans does.
    #[derive(Default)]
    struct Recorder {
        events: AtomicUsize,
        spans: Mutex<Vec<&'static Metadata<'static>>>,
        current: Mutex<Option<u64>>,
        entered: Mutex<Vec<(u64, Option<String>)>>,
    }

    /// A subscriber writing to a shared [`Recorder`].
    struct Recording(Arc<Recorder>);

    impl Subscriber for Recording {
        fn enabled(&self, _: &Metadata<'_>) -> bool {
            true
        }
        fn new_span(&self, attributes: &Attributes<'_>) -> Id {
            let mut spans = self.0.spans.lock().unwrap();
            spans.push(attributes.metadata());
            Id::from_u64(spans.len() as u64)
        }
        fn record(&self, _: &Id, _: &Record<'_>) {}
        fn record_follows_from(&self, _: &Id, _: &Id) {}
        fn event(&self, _: &Event<'_>) {
            self.0.events.fetch_add(1, Ordering::SeqCst);
        }
        fn enter(&self, span: &Id) {
            let thread = std::thread::current().name().map(str::to_owned);
            self.0
                .entered
                .lock()
                .unwrap()
                .push((span.into_u64(), thread));
            *self.0.current.lock().unwrap() = Some(span.into_u64());
        }
        fn exit(&self, _: &Id) {
            *self.0.current.lock().unwrap() = None;
        }
        fn current_span(&self) -> Current {
            let current = *self.0.current.lock().unwrap();
            current.map_or_else(Current::none, |id| {
                let metadata = self.0.spans.lock().unwrap()[usize::try_from(id - 1).unwrap()];
                Current::new(Id::from_u64(id), metadata)
            })
        }
    }

    #[test]
    fn the_loader_logs_to_the_callers_subscriber_within_the_callers_span() {
        let recorder = Arc::new(Recorder::default());
        tracing::subscriber::with_default(Recording(Arc::clone(&recorder)), || {
            let span = tracing::info_span!("caller");
            let _entered = span.enter();
            on_large_stack(|| tracing::warn!("from the loader"));
        });
        assert_eq!(recorder.events.load(Ordering::SeqCst), 1);
        let entered = recorder.entered.lock().unwrap().clone();
        assert!(
            entered.contains(&(1, Some("sim-mjcf-loader".to_owned()))),
            "{entered:?}"
        );
    }
}
