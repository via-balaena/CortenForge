//! Native file dialogs — and the only safe way to open one from a Bevy system.
//!
//! ## ⚠⚠ Never call a blocking `rfd` dialog from a system
//!
//! `rfd::FileDialog::pick_file()` / `pick_folder()` block, and blocking from a
//! Bevy system **deadlocks the app on macOS**. Both halves were sampled:
//!
//! - the system runs on a task-pool **worker**, where rfd's `run_on_main`
//!   `dispatch_sync`s the modal panel to the main thread (macOS requires modal
//!   UI there);
//! - the **main thread** is already parked in
//!   `TaskPool::scope_with_executor_inner`, waiting for that very system.
//!
//! Neither side can move. It is deterministic, not a race — it fires every time.
//! Slint's event loop had no ECS task pool, which is why the pre-port code could
//! call these directly.
//!
//! The fix, measured: spawn the **async** dialog on the task pool and poll it,
//! so the main thread stays free to service the dispatch. The app kept a full
//! 60 fps with a dialog open.

use std::path::PathBuf;

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};

/// The scan formats the picker offers. `cortenforge` enables `mesh-io/threemf`,
/// so all four reach a loader.
const SCAN_EXTENSIONS: &[&str] = &["stl", "obj", "ply", "3mf"];

/// What a layer design is saved as — `<scan>.design.toml`, so the filter is
/// plain TOML.
const DESIGN_EXTENSIONS: &[&str] = &["toml"];

/// What a dialog was opened for, so the poller can route the chosen path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DialogKind {
    /// Step 1 (Add scan): the scan file to work from.
    ScanFile,
    /// Step 6 (Print): the folder the printable files are copied into.
    PrintDest,
    /// Step 2 (Save): a folder other than the scan's own to write the cleaned
    /// scan + `.prep.toml` into, chosen because the default already held them.
    PrepDest,
    /// Step 4 (Design layers): a `.design.toml` to use instead of the stack
    /// built in the editor.
    DesignFile,
}

/// The at-most-one dialog currently open.
///
/// One at a time is deliberate: two OS pickers racing to write the same slot
/// would drop one result silently, and there is no screen that needs two.
#[derive(Resource, Default)]
pub(crate) struct PendingDialog(Option<(DialogKind, Task<Option<PathBuf>>)>);

impl PendingDialog {
    /// Whether a dialog is open — panels disable their buttons on this.
    pub(crate) const fn is_open(&self) -> bool {
        self.0.is_some()
    }

    /// A [`PendingDialog`] that reports itself open, so a test can prove a
    /// control is gated on one without putting an OS picker on screen. The task
    /// resolves straight to a cancel.
    #[cfg(test)]
    pub(crate) fn opened(kind: DialogKind) -> Self {
        Self::resolved(kind, None)
    }

    /// A [`PendingDialog`] whose task is already answered, so a test can drive
    /// [`Self::poll`]'s consumers down either branch — a chosen path, or the
    /// `None` that means the user cancelled.
    #[cfg(test)]
    pub(crate) fn resolved(kind: DialogKind, picked: Option<PathBuf>) -> Self {
        let pool = AsyncComputeTaskPool::get_or_init(bevy::tasks::TaskPool::default);
        Self(Some((kind, pool.spawn(async move { picked }))))
    }

    /// Open a folder picker for `kind`. A no-op while one is already open.
    pub(crate) fn pick_folder(&mut self, kind: DialogKind, title: &'static str) {
        if self.0.is_some() {
            return;
        }
        let task = AsyncComputeTaskPool::get().spawn(async move {
            rfd::AsyncFileDialog::new()
                .set_title(title)
                .pick_folder()
                .await
                .map(|handle| handle.path().to_path_buf())
        });
        self.0 = Some((kind, task));
    }

    /// Open a file picker for `kind`, offering `extensions` under `filter`.
    /// A no-op while one is already open.
    ///
    /// ⚠ Untestable: calling it opens a real OS picker, and pre-opening one to
    /// stop that hits the early return below. `cargo-mutants` empties this and
    /// both its wrappers to `()` and the suite stays green. `poll_dialogs`
    /// routes on `kind`, so a swapped one sends a chosen `.design.toml` down
    /// the scan path, where it fails to load and resets the project.
    fn pick_file(
        &mut self,
        kind: DialogKind,
        title: &'static str,
        filter: &'static str,
        extensions: &'static [&'static str],
    ) {
        if self.0.is_some() {
            return;
        }
        let task = AsyncComputeTaskPool::get().spawn(async move {
            rfd::AsyncFileDialog::new()
                .set_title(title)
                .add_filter(filter, extensions)
                .pick_file()
                .await
                .map(|handle| handle.path().to_path_buf())
        });
        self.0 = Some((kind, task));
    }

    /// Open a scan-file picker.
    pub(crate) fn pick_scan_file(&mut self) {
        self.pick_file(
            DialogKind::ScanFile,
            "Choose your 3D scan",
            "3D scan",
            SCAN_EXTENSIONS,
        );
    }

    /// Open a design-file picker.
    pub(crate) fn pick_design_file(&mut self) {
        self.pick_file(
            DialogKind::DesignFile,
            "Choose the design file",
            "design",
            DESIGN_EXTENSIONS,
        );
    }

    /// Take the result if the dialog has resolved. `Some((kind, None))` means
    /// the user cancelled — a real outcome, distinct from "still open".
    pub(crate) fn poll(&mut self) -> Option<(DialogKind, Option<PathBuf>)> {
        let (kind, task) = self.0.as_mut()?;
        let kind = *kind;
        let picked = future::block_on(future::poll_once(task))?;
        self.0 = None;
        Some((kind, picked))
    }
}
